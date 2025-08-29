// main.cpp
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "third_party/json/single_include/nlohmann/json.hpp"
#include "utils.h"
#include "types.h"
#include "first_solution_parser.h"
#include "first_schedule_solver.h"   // use header (for SolveOptions + API)

using json = nlohmann::json;
namespace fs = std::filesystem;

// ---------------- Minimal CLI ----------------
struct Flags {
  std::string mode = "all";       // {init|alns|all}
  std::string tasks_path;         // required
  std::string matrix_path;        // required
  std::string config_path;        // required
  std::string seeds_in_dir = "";  // optional warm-start pool
  bool verbose = true;            // flipped by --quiet
};

static void print_usage() {
  std::cout <<
R"(Usage:
  solver --mode {init|alns|all} --tasks tasks.json --matrix matrix.json --config config.json [--seeds_in DIR] [--quiet]

Required:
  --mode {init|alns|all}
  --tasks PATH
  --matrix PATH
  --config PATH

Optional:
  --seeds_in DIR      Load warm-start seeds (JSON files) from directory
  --quiet             Less logging
  --help
)";
}

static Flags parse_flags(int argc, char** argv) {
  Flags f;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto need = [&](const char* name) {
      if (i + 1 >= argc) { std::cerr << "Missing value for " << name << "\n"; std::exit(2); }
      return std::string(argv[++i]);
    };
    if (a == "--help" || a == "-h") { print_usage(); std::exit(0); }
    else if (a == "--mode")   f.mode = need("--mode");
    else if (a == "--tasks")  f.tasks_path = need("--tasks");
    else if (a == "--matrix") f.matrix_path = need("--matrix");
    else if (a == "--config") f.config_path = need("--config");
    else if (a == "--seeds_in") f.seeds_in_dir = need("--seeds_in");
    else if (a == "--quiet") f.verbose = false;
    else { std::cerr << "Unknown flag: " << a << "\n"; print_usage(); std::exit(2); }
  }
  if (f.tasks_path.empty() || f.matrix_path.empty() || f.config_path.empty()) {
    std::cerr << "Missing required --tasks/--matrix/--config.\n"; print_usage(); std::exit(2);
  }
  return f;
}

// ---------------- Small utils ----------------
static json load_json(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("Cannot open file: " + path);
  json j; in >> j; return j;
}
static void save_json(const std::string& path, const json& j) {
  fs::create_directories(fs::path(path).parent_path());
  std::ofstream out(path);
  if (!out) throw std::runtime_error("Cannot write file: " + path);
  out << std::setw(2) << j << "\n";
}
static std::vector<std::string> list_json_files(const std::string& dir) {
  std::vector<std::string> res;
  if (dir.empty() || !fs::exists(dir)) return res;
  for (auto& p : fs::directory_iterator(dir)) {
    if (p.is_regular_file() && p.path().extension() == ".json") res.push_back(p.path().string());
  }
  std::sort(res.begin(), res.end());
  return res;
}
static std::string timestamp() {
  auto t = std::time(nullptr); std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  std::ostringstream oss; oss << std::put_time(&tm, "%Y%m%d-%H%M%S"); return oss.str();
}
static std::string seed_filename(const std::string& out_dir, int run_idx, int vehicles) {
  std::ostringstream oss; oss << out_dir << "/seed_" << vehicles << "veh_run" << run_idx << "_" << timestamp() << ".json";
  return oss.str();
}

// ---------------- Config helpers ----------------
struct Cfg {
  // relaxed seeding
  int relaxed_vehicles;
  int relaxed_runs;
  int relaxed_time_limit_s;
  int threads;
  int rng_base_seed;
  std::string seeds_out_dir;

  // target + ALNS
  int target_vehicles;
  int alns_time_limit_s;
  int alns_moves;
  double vehicle_penalty_alpha;
  bool log_progress;
  std::string result_out;

  // pass-through base config JSON (to feed into first_schedule_solver)
  json base;
};

static Cfg parse_config(const json& j) {
  const unsigned def_threads_u = std::max(1u, std::thread::hardware_concurrency());
  const int def_threads = (def_threads_u > static_cast<unsigned>(std::numeric_limits<int>::max()))
      ? std::numeric_limits<int>::max()
      : static_cast<int>(def_threads_u);

  Cfg c{
    /*relaxed_vehicles*/      j.value("RELAXED_VEHICLES", 26),
    /*relaxed_runs*/          j.value("RELAXED_RUNS", 4),
    /*relaxed_time_limit_s*/  j.value("RELAXED_TIME_LIMIT_SECONDS", 1800),
    /*threads*/               j.value("THREADS", def_threads),
    /*rng_base_seed*/         j.value("RNG_BASE_SEED", 12345),
    /*seeds_out_dir*/         j.value("SEEDS_OUT_DIR", std::string("seeds")),
    /*target_vehicles*/       j.value("TARGET_VEHICLES", 25),
    /*alns_time_limit_s*/     j.value("ALNS_TIME_LIMIT_SECONDS", 3600),
    /*alns_moves*/            j.value("ALNS_MOVES", 20000),
    /*vehicle_penalty_alpha*/ j.value("ALPHA", 0.0),
    /*log_progress*/          j.value("LOG_PROGRESS", true),
    /*result_out*/            j.value("RESULT_OUT", std::string("result.json")),
    /*base*/                  j
  };
  return c;
}

// ---------------- Optional ALNS hook ----------------
namespace inspiration {
  nlohmann::json run_alns(const nlohmann::json& tasks,
                          const nlohmann::json& matrix,
                          const nlohmann::json& base_config,
                          const std::vector<nlohmann::json>& seed_solutions,
                          const nlohmann::json& alns_params);
}

// ---------------- Seeding (parallel relaxed runs) ----------------
static std::vector<json> run_relaxed_seeds_parallel(const Cfg& c,
                                                    bool verbose,
                                                    const json& tasks,
                                                    const json& matrix,
                                                    json base_config) {
  // Tag (optional) and uppercase keys (if solver ever reads from JSON),
  // but we *enforce* limits via SolveOptions below.
  base_config["RELAXED_FIRST_PASS"] = true;
  base_config["VEHICLES_PER_DAY"]   = c.relaxed_vehicles;     // harmless duplicate with opts
  base_config["TIME_LIMIT_SECONDS"] = c.relaxed_time_limit_s; // harmless duplicate with opts

  if (verbose) {
    std::cout << "🧭 Relaxed seeding:"
              << " runs=" << c.relaxed_runs
              << " threads=" << c.threads
              << " veh=" << c.relaxed_vehicles
              << " tlimit=" << c.relaxed_time_limit_s << "s\n";
  }

  fs::create_directories(c.seeds_out_dir);

  std::atomic<int> next_idx{0};
  std::mutex io_mu;
  std::vector<std::future<void>> pool;
  std::vector<json> seeds(c.relaxed_runs);

  const int max_threads = std::max(1, std::min(c.threads, c.relaxed_runs));
  for (int t = 0; t < max_threads; ++t) {
    pool.emplace_back(std::async(std::launch::async, [&]() {
      for (;;) {
        int i = next_idx.fetch_add(1);
        if (i >= c.relaxed_runs) break;

        json cfg_i = base_config;

        // Enforce limits via SolveOptions (bullet-proof)
        first_schedule_solver::SolveOptions so{};
        so.log_search             = verbose;
        so.vehicles_per_day       = c.relaxed_vehicles;
        so.time_limit_seconds     = c.relaxed_time_limit_s;
        so.random_seed            = c.rng_base_seed + i;
        so.enable_operator_bundle = true;          // keep LS rich on long runs
        so.overtime_cap_minutes     = 45;   // try 30–60 for relaxed seeding
        so.overtime_soft_coeff      = 1;    // small penalty per minute overtime
        // Optional extra knobs (uncomment if desired):
        // so.lns_time_limit_seconds = c.relaxed_time_limit_s; // per-LNS cap
        // so.log_search = verbose;

        auto t0 = NowMillis();
        json sol;
        try {
          sol = first_schedule_solver::solve_problem(tasks, matrix, cfg_i, so);
        } catch (const std::exception& e) {
          std::lock_guard<std::mutex> lk(io_mu);
          std::cerr << "Relaxed run " << i << " failed: " << e.what() << "\n";
          continue;
        }
        auto t1 = NowMillis();

        sol["meta"]["relaxed_run_index"] = i;
        sol["meta"]["elapsed_ms"] = static_cast<long long>(t1 - t0);
        sol["meta"]["vehicles_used"] = c.relaxed_vehicles;

        const std::string out_path = seed_filename(c.seeds_out_dir, i, c.relaxed_vehicles);
        try { save_json(out_path, sol); }
        catch (const std::exception& e) {
          std::lock_guard<std::mutex> lk(io_mu);
          std::cerr << "Failed to save seed " << i << ": " << e.what() << "\n";
        }

        {
          std::lock_guard<std::mutex> lk(io_mu);
          if (verbose) std::cout << "   ✓ Seed " << i << " (" << (t1 - t0) << " ms) -> " << out_path << "\n";
        }
        seeds[i] = std::move(sol);
      }
    }));
  }
  for (auto& fut : pool) fut.get();

  std::vector<json> out;
  out.reserve(seeds.size());
  for (auto& s : seeds) if (!s.is_null() && !s.empty()) out.push_back(std::move(s));
  return out;
}

static std::vector<json> load_seeds_from_dir(const std::string& dir, bool verbose) {
  std::vector<json> res;
  for (const auto& f : list_json_files(dir)) {
    try { res.push_back(load_json(f)); if (verbose) std::cout << "   • loaded seed: " << f << "\n"; }
    catch (const std::exception& e) { std::cerr << "   ! failed to load " << f << ": " << e.what() << "\n"; }
  }
  return res;
}

// ---------------- Main ----------------
int main(int argc, char** argv) {
  std::ios::sync_with_stdio(false);
  const Flags flags = parse_flags(argc, argv);

  if (flags.verbose) {
    std::cout << "🚀 VRP Van Laar — init + ALNS\n";
    std::cout << "Mode: " << flags.mode << "\n";
  }

  json tasks, matrix, cfg_json;
  try {
    tasks = load_json(flags.tasks_path);
    matrix = load_json(flags.matrix_path);
    cfg_json = load_json(flags.config_path);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load inputs: " << e.what() << "\n"; return 1;
  }

  const Cfg cfg = parse_config(cfg_json);
  if (flags.verbose) {
    std::cout << "Config: relaxed_veh=" << cfg.relaxed_vehicles
              << " runs=" << cfg.relaxed_runs
              << " threads=" << cfg.threads
              << " target_veh=" << cfg.target_vehicles << "\n";
  }

  // Warm pool (optional)
  std::vector<json> seed_pool;
  if (!flags.seeds_in_dir.empty()) {
    if (flags.verbose) std::cout << "📂 Loading warm-start seeds from: " << flags.seeds_in_dir << "\n";
    auto loaded = load_seeds_from_dir(flags.seeds_in_dir, flags.verbose);
    seed_pool.insert(seed_pool.end(), loaded.begin(), loaded.end());
  }

  // INIT
  if (flags.mode == "init" || flags.mode == "all") {
    auto new_seeds = run_relaxed_seeds_parallel(cfg, flags.verbose, tasks, matrix, cfg.base);
    if (new_seeds.empty()) {
      std::cerr << "No seeds produced in INIT.\n";
      if (flags.mode == "init") return 2;
    }
    seed_pool.insert(seed_pool.end(), new_seeds.begin(), new_seeds.end());
    if (flags.verbose) std::cout << "📦 Seed pool size after INIT: " << seed_pool.size() << "\n";
  }

  // ALNS
  if (flags.mode == "alns" || flags.mode == "all") {
    if (seed_pool.empty() && flags.verbose) {
      std::cout << "⚠️  No seeds available. Proceeding with empty pool; ALNS should self-seed.\n";
    }

    json alns_params = {
      {"time_limit_seconds", cfg.alns_time_limit_s},
      {"max_moves", cfg.alns_moves},
      {"target_vehicles", cfg.target_vehicles},
      {"vehicle_penalty_alpha", cfg.vehicle_penalty_alpha},
      {"log_progress", cfg.log_progress},
      {"ramp_schedule", {{"alpha_step", 0.25}, {"every_moves", 2000}}},
      {"overtime_window_start_min", 60},
      {"overtime_window_end_min", 0}
    };

    if (flags.verbose) {
      std::cout << "🧩 ALNS starting..."
                << " time=" << cfg.alns_time_limit_s << "s"
                << " moves=" << cfg.alns_moves
                << " target_veh=" << cfg.target_vehicles << "\n";
    }

    json final_result;
    try {
      // Hook up when ready:
      // final_result = inspiration::run_alns(tasks, matrix, cfg.base, seed_pool, alns_params);
      final_result = { {"meta", {{"note","ALNS not yet implemented"}}} };
    } catch (const std::exception& e) {
      std::cerr << "ALNS failed: " << e.what() << "\n"; return 3;
    }

    try { save_json(cfg.result_out, final_result); }
    catch (const std::exception& e) { std::cerr << "Failed to write result: " << e.what() << "\n"; return 4; }

    if (flags.verbose) std::cout << "✅ Result written to " << cfg.result_out << "\n";
  }

  if (flags.verbose) std::cout << "🏁 Done.\n";
  return 0;
}
