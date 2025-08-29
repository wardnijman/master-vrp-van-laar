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
#include "first_schedule_solver.h" // SolveOptions + API

// Seeds & generators
#include "seed.h"
#include "seed_generators.h"

using json = nlohmann::json;
namespace fs = std::filesystem;

// ---------- CLI ----------
struct Flags
{
  std::string mode = "all";      // {init|alns|all}
  std::string tasks_path;        // required
  std::string matrix_path;       // required
  std::string config_path;       // required
  std::string seeds_in_dir = ""; // optional warm-start pool of relaxed JSONs
  bool verbose = true;           // flipped by --quiet
};

static void print_usage()
{
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

static Flags parse_flags(int argc, char **argv)
{
  Flags f;
  for (int i = 1; i < argc; ++i)
  {
    std::string a = argv[i];
    auto need = [&](const char *name)
    {
      if (i + 1 >= argc)
      {
        std::cerr << "Missing value for " << name << "\n";
        std::exit(2);
      }
      return std::string(argv[++i]);
    };
    if (a == "--help" || a == "-h")
    {
      print_usage();
      std::exit(0);
    }
    else if (a == "--mode")
      f.mode = need("--mode");
    else if (a == "--tasks")
      f.tasks_path = need("--tasks");
    else if (a == "--matrix")
      f.matrix_path = need("--matrix");
    else if (a == "--config")
      f.config_path = need("--config");
    else if (a == "--seeds_in")
      f.seeds_in_dir = need("--seeds_in");
    else if (a == "--quiet")
      f.verbose = false;
    else
    {
      std::cerr << "Unknown flag: " << a << "\n";
      print_usage();
      std::exit(2);
    }
  }
  if (f.tasks_path.empty() || f.matrix_path.empty() || f.config_path.empty())
  {
    std::cerr << "Missing required --tasks/--matrix/--config.\n";
    print_usage();
    std::exit(2);
  }
  return f;
}

// ---------- Small utils ----------
static json load_json(const std::string &path)
{
  std::ifstream in(path);
  if (!in)
    throw std::runtime_error("Cannot open file: " + path);
  json j;
  in >> j;
  return j;
}
static void save_json(const std::string &path, const json &j)
{
  fs::create_directories(fs::path(path).parent_path());
  std::ofstream out(path);
  if (!out)
    throw std::runtime_error("Cannot write file: " + path);
  out << std::setw(2) << j << "\n";
}
static std::vector<std::string> list_json_files(const std::string &dir)
{
  std::vector<std::string> res;
  if (dir.empty() || !fs::exists(dir))
    return res;
  for (auto &p : fs::directory_iterator(dir))
  {
    if (p.is_regular_file() && p.path().extension() == ".json")
      res.push_back(p.path().string());
  }
  std::sort(res.begin(), res.end());
  return res;
}
static std::string timestamp()
{
  auto t = std::time(nullptr);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d-%H%M%S");
  return oss.str();
}
static std::string seed_filename(const std::string &out_dir, const std::string &tag, int idx)
{
  std::ostringstream oss;
  oss << out_dir << "/" << tag << "_" << std::setw(2) << std::setfill('0') << idx
      << "_" << timestamp() << ".json";
  return oss.str();
}

// Return a const ref to the array of tasks (either plain array, or object{"tasks":[...]})
static const json &tasks_array_view(const json &tasks_json)
{
  if (tasks_json.is_array())
    return tasks_json;
  if (tasks_json.is_object())
  {
    auto it = tasks_json.find("tasks");
    if (it != tasks_json.end() && it->is_array())
      return *it;
  }
  throw std::runtime_error("tasks.json must be a JSON array or an object with a 'tasks' array.");
}

// ---------- Config ----------
struct Cfg
{
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

  // heuristic seed build
  std::vector<std::string> seed_generators;
  int seed_trials_per_gen;
  int seed_keep_per_gen;
  int seed_diverse_k;
  std::string heuristic_seeds_out_dir;

  // pass-through base config JSON (to feed into first_schedule_solver)
  json base;
};

static Cfg parse_config(const json &j)
{
  const unsigned def_threads_u = std::max(1u, std::thread::hardware_concurrency());
  const int def_threads = (def_threads_u > static_cast<unsigned>(std::numeric_limits<int>::max()))
                              ? std::numeric_limits<int>::max()
                              : static_cast<int>(def_threads_u);

  Cfg c{
      /*relaxed_vehicles*/ j.value("RELAXED_VEHICLES", 26),
      /*relaxed_runs*/ j.value("RELAXED_RUNS", 4),
      /*relaxed_time_limit_s*/ j.value("RELAXED_TIME_LIMIT_SECONDS", 1800),
      /*threads*/ j.value("THREADS", def_threads),
      /*rng_base_seed*/ j.value("RNG_BASE_SEED", 12345),
      /*seeds_out_dir*/ j.value("SEEDS_OUT_DIR", std::string("seeds")),
      /*target_vehicles*/ j.value("TARGET_VEHICLES", 25),
      /*alns_time_limit_s*/ j.value("ALNS_TIME_LIMIT_SECONDS", 3600),
      /*alns_moves*/ j.value("ALNS_MOVES", 20000),
      /*vehicle_penalty_alpha*/ j.value("ALPHA", 0.0),
      /*log_progress*/ j.value("LOG_PROGRESS", true),
      /*result_out*/ j.value("RESULT_OUT", std::string("result.json")),
      /*seed_generators*/ j.value("SEED_GENERATORS", std::vector<std::string>{"two_week_rollout", "capacity_balanced", "sweep"}),
      /*seed_trials_per_gen*/ j.value("SEED_TRIALS_PER_GEN", 5),
      /*seed_keep_per_gen*/ j.value("SEED_KEEP_PER_GEN", 3),
      /*seed_diverse_k*/ j.value("SEED_DIVERSE_K", 8),
      /*heuristic_seeds_out_dir*/ j.value("HEURISTIC_SEEDS_OUT_DIR", std::string("heuristic_seeds")),
      /*base*/ j};
  return c;
}

// ---------- Optional ALNS hook ----------
namespace inspiration
{
  nlohmann::json run_alns(const nlohmann::json &tasks,
                          const nlohmann::json &matrix,
                          const nlohmann::json &base_config,
                          const std::vector<nlohmann::json> &seed_solutions,
                          const nlohmann::json &alns_params);
}

// ---------- FIRST-PASS relaxed seeds (parallel) ----------
static std::vector<json> run_relaxed_seeds_parallel(const Cfg &c,
                                                    bool verbose,
                                                    const json &tasks,
                                                    const json &matrix,
                                                    json base_config)
{
  base_config["RELAXED_FIRST_PASS"] = true;
  base_config["VEHICLES_PER_DAY"] = c.relaxed_vehicles;
  base_config["TIME_LIMIT_SECONDS"] = c.relaxed_time_limit_s;

  if (verbose)
  {
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
  for (int t = 0; t < max_threads; ++t)
  {
    pool.emplace_back(std::async(std::launch::async, [&]()
                                 {
      for (;;) {
        int i = next_idx.fetch_add(1);
        if (i >= c.relaxed_runs) break;

        json cfg_i = base_config;

        first_schedule_solver::SolveOptions so{};
        so.vehicles_per_day       = c.relaxed_vehicles;
        so.time_limit_seconds     = c.relaxed_time_limit_s;
        so.random_seed            = c.rng_base_seed + i;
        so.enable_operator_bundle = true;

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

        const std::string out_path = seed_filename(c.seeds_out_dir, "seed_relaxed", i);
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
      } }));
  }
  for (auto &fut : pool)
    fut.get();

  std::vector<json> out;
  out.reserve(seeds.size());
  for (auto &s : seeds)
    if (!s.is_null() && !s.empty())
      out.push_back(std::move(s));
  return out;
}

// ---------- Tasks & Matrix helpers ----------
static std::vector<Task> load_tasks_into_seed_structs(const json &tasks_json_all)
{
  const json &arr = tasks_array_view(tasks_json_all);
  std::vector<Task> out;
  out.reserve(arr.size());
  for (const auto &t : arr)
  {
    Task tt{};
    tt.task_id = t.value("task_id", "");
    tt.schema_number = t.value("schema_number", 0);
    tt.client_id = t.value("client_id", 0);
    tt.duration = (double)t.value("duration", 0);
    // tolerate either frequency.week or top-level week
    if (t.contains("frequency") && t["frequency"].is_object())
    {
      tt.week = t["frequency"].value("week", t.value("week", 0));
    }
    else
    {
      tt.week = t.value("week", 0);
    }
    // day_options: expect [{date, weekday}]
    if (t.contains("day_options") && t["day_options"].is_array())
    {
      for (const auto &o : t["day_options"])
      {
        Task::DayOption d{};
        d.date = o.value("date", "");
        d.weekday = o.value("weekday", 0);
        tt.day_options.push_back(d);
      }
    }
    out.push_back(std::move(tt));
  }
  return out;
}

// If your input matrix is in SECONDS, DIV = 60. If already minutes, DIV = 1.
static constexpr int DIV = 60;

static TimeMatrix load_minutes_matrix(const nlohmann::json &matrix_json)
{
  TimeMatrix M;
  M.n = static_cast<int>(matrix_json.size());
  if (M.n <= 0)
    return M;
  M.m.resize(static_cast<size_t>(M.n) * static_cast<size_t>(M.n));
  for (int i = 0; i < M.n; ++i)
  {
    std::vector<int> row = matrix_json[i].get<std::vector<int>>();
    if ((int)row.size() != M.n)
    {
      throw std::runtime_error("Distance matrix is not square at row " + std::to_string(i));
    }
    for (int j = 0; j < M.n; ++j)
    {
      int v = row[j] / DIV;
      if (v < 0)
        v = 0;
      M.m[static_cast<size_t>(i) * M.n + j] = v;
    }
  }
  return M;
}

// ---------- Relaxed JSON -> Seed (robust) ----------
static Seed seed_from_relaxed_json(const json &sol_json, const std::vector<Task> &original_tasks)
{
  // Pull chosen date per task
  std::unordered_map<std::string, std::string> chosen_date;

  // Preferred: explicit assigned list
  if (sol_json.contains("assigned") && sol_json["assigned"].is_array())
  {
    for (const auto &a : sol_json["assigned"])
    {
      std::string tid = a.value("task_id", "");
      std::string d = a.value("date", "");
      if (!tid.empty() && !d.empty())
        chosen_date[tid] = d;
    }
  }

  // Fallback: routes_by_day = array[day][vehicle][task_id string]
  if (chosen_date.empty() && sol_json.contains("routes_by_day") && sol_json["routes_by_day"].is_array())
  {
    std::vector<std::string> dates;
    if (sol_json.contains("dates") && sol_json["dates"].is_array())
    {
      for (const auto &s : sol_json["dates"])
        dates.push_back(s.get<std::string>());
    }
    // If no "dates" provided, we can’t reconstruct reliable dates; skip this fallback.
    if (!dates.empty())
    {
      const auto &rbd = sol_json["routes_by_day"];
      int D = (int)rbd.size();
      for (int d = 0; d < D && d < (int)dates.size(); ++d)
      {
        if (!rbd[d].is_array())
          continue;
        for (const auto &route : rbd[d])
        {
          if (!route.is_array())
            continue;
          for (const auto &tid_json : route)
          {
            if (!tid_json.is_string())
              continue;
            chosen_date[tid_json.get<std::string>()] = dates[d];
          }
        }
      }
    }
  }

  auto choose_one = [](const Task &t, const Task::DayOption *opt)
  {
    Task out = t;
    out.day_options.clear();
    if (opt)
      out.day_options.push_back(*opt);
    return out;
  };

  Seed s;
  s.name = "relaxed_json";
  s.tasks.reserve(original_tasks.size());
  for (const auto &t : original_tasks)
  {
    const Task::DayOption *picked = nullptr;
    if (auto it = chosen_date.find(t.task_id); it != chosen_date.end())
    {
      for (const auto &o : t.day_options)
        if (o.date == it->second)
        {
          picked = &o;
          break;
        }
    }
    if (!picked && !t.day_options.empty())
      picked = &t.day_options.front();
    if (picked)
      s.tasks.push_back(choose_one(t, picked));
  }
  return s;
}

// ---------- Seed JSON writer (compact) ----------
static json seed_to_json(const Seed &s)
{
  json j;
  j["name"] = s.name;

  std::vector<json> arr;
  arr.reserve(s.tasks.size());

  for (const auto &t : s.tasks)
  {
    json jt;
    jt["task_id"] = t.task_id;
    jt["schema_number"] = t.schema_number;
    jt["client_id"] = t.client_id;
    jt["duration"] = t.duration;
    jt["week"] = t.week;
    if (!t.day_options.empty())
    {
      jt["date"] = t.day_options.front().date;
      jt["weekday"] = t.day_options.front().weekday;
    }
    arr.push_back(std::move(jt));
  }
  j["tasks"] = std::move(arr);
  return j;
}

// ---------- Heuristic seed build + save ----------
static std::vector<RankedSeed> build_and_save_heuristic_seeds(
    const Cfg &cfg,
    bool verbose,
    const json &tasks_json_all,
    const json &matrix_json,
    const std::vector<json> &relaxed_seed_jsons)
{
  if (verbose)
    std::cout << "🌱 Building heuristic seeds...\n";

  // Convert inputs to evaluation types
  std::vector<Task> base_tasks = load_tasks_into_seed_structs(tasks_json_all);
  TimeMatrix M = load_minutes_matrix(matrix_json);

  // All config numbers come from cfg.base (never from tasks_json which is an array)
  EvalConfig ecfg{};
  ecfg.matrix = &M;
  ecfg.max_hours_per_route = cfg.base.value("MAX_HOURS_PER_DAY", 9);
  ecfg.max_routes_per_day = cfg.target_vehicles;
  ecfg.day_cfg.vehicles = cfg.target_vehicles;
  ecfg.day_cfg.route_limit_minutes = ecfg.max_hours_per_route * 60;
  ecfg.explore_params.time_limit_seconds = 2; // quick per-day exploratory solve
  ecfg.explore_params.log_search = false;

  // Give generators access to M
  seed_generators_set_matrix(&M);

  // 1) Heuristic candidates via your seed.cpp helper
  int K = std::max(cfg.seed_diverse_k, 1);
  register_default_seed_generators();
  
  auto heuristic_ranked = build_and_rank_seeds(cfg.seed_generators, base_tasks, ecfg, K);

  // 2) Add relaxed seeds from disk, score, and merge
  std::vector<RankedSeed> relaxed_ranked;
  relaxed_ranked.reserve(relaxed_seed_jsons.size());
  for (const auto &sj : relaxed_seed_jsons)
  {
    Seed s = seed_from_relaxed_json(sj, base_tasks);

    // 🔧 Run through the "raw" generator to make weekday encoding match the validator.
    // "raw" keeps the chosen date per task, but sets weekday accordingly for validation.
    if (SeedRegistry::has("raw"))
    {
      Seed s_iso = SeedRegistry::get("raw")(s.tasks);
      s = std::move(s_iso);
    }

    s.name = "relaxed_cached";
    RankedSeed r;
    r.seed = std::move(s);

    if (auto msg = run_validation(r.seed.tasks); !msg.empty())
    {
      r.score.valid = false;
      r.score.validation_error = msg;
      r.score.total_cost = 1e18;
    }
    else
    {
      r.score = evaluate_seed_cost(r.seed, ecfg);
      r.score.valid = true;
    }
    relaxed_ranked.push_back(std::move(r));
  }

  // Merge and re-sort by total_cost
  heuristic_ranked.insert(heuristic_ranked.end(), relaxed_ranked.begin(), relaxed_ranked.end());
  sort_by_cost(heuristic_ranked);

  // Keep top K overall
  if ((int)heuristic_ranked.size() > K)
    heuristic_ranked.resize(K);

  // 3) Save and log
  fs::create_directories(cfg.heuristic_seeds_out_dir);
  int idx = 0;
  if (verbose)
    std::cout << "📊 Seed quality (lower is better):\n";
  for (const auto &r : heuristic_ranked)
  {
    json j = seed_to_json(r.seed);
    j["score"] = {
        {"valid", r.score.valid},
        {"total_cost", r.score.total_cost},
        {"validation_error", r.score.validation_error}};
    auto path = seed_filename(cfg.heuristic_seeds_out_dir, "heuristic_seed", idx);
    save_json(path, j);
    if (verbose)
    {
      std::cout << "   #" << std::setw(2) << idx
                << "  cost=" << std::fixed << std::setprecision(2) << r.score.total_cost
                << "  name=" << r.seed.name
                << "  -> " << path << "\n";
    }
    ++idx;
  }
  if (verbose)
    std::cout << "🌿 Heuristic seeds saved: " << heuristic_ranked.size()
              << " → " << cfg.heuristic_seeds_out_dir << "\n";

  return heuristic_ranked;
}

// ---------- Main ----------
int main(int argc, char **argv)
{
  std::ios::sync_with_stdio(false);
  const Flags flags = parse_flags(argc, argv);

  if (flags.verbose)
  {
    std::cout << "🚀 VRP Van Laar — init + seeds + ALNS\n";
    std::cout << "Mode: " << flags.mode << "\n";
  }

  json tasks, matrix, cfg_json;
  try
  {
    tasks = load_json(flags.tasks_path);
    matrix = load_json(flags.matrix_path);
    cfg_json = load_json(flags.config_path);
  }
  catch (const std::exception &e)
  {
    std::cerr << "Failed to load inputs: " << e.what() << "\n";
    return 1;
  }

  const Cfg cfg = parse_config(cfg_json);
  if (flags.verbose)
  {
    std::cout << "Config: relaxed_veh=" << cfg.relaxed_vehicles
              << " runs=" << cfg.relaxed_runs
              << " threads=" << cfg.threads
              << " target_veh=" << cfg.target_vehicles << "\n";
  }

  // Warm pool (optional): relaxed seed JSONs
  std::vector<json> relaxed_seed_jsons;
  if (!flags.seeds_in_dir.empty())
  {
    if (flags.verbose)
      std::cout << "📂 Loading warm-start relaxed seeds from: " << flags.seeds_in_dir << "\n";
    for (const auto &f : list_json_files(flags.seeds_in_dir))
    {
      try
      {
        relaxed_seed_jsons.push_back(load_json(f));
        if (flags.verbose)
          std::cout << "   • loaded: " << f << "\n";
      }
      catch (const std::exception &e)
      {
        std::cerr << "   ! failed " << f << ": " << e.what() << "\n";
      }
    }
  }

  // INIT (relaxed parallel runs)
  if (flags.mode == "init" || flags.mode == "all")
  {
    auto new_seeds = run_relaxed_seeds_parallel(cfg, flags.verbose, tasks, matrix, cfg.base);
    if (new_seeds.empty())
    {
      std::cerr << "No seeds produced in INIT.\n";
      if (flags.mode == "init")
        return 2;
    }
    relaxed_seed_jsons.insert(relaxed_seed_jsons.end(), new_seeds.begin(), new_seeds.end());
    if (flags.verbose)
      std::cout << "📦 Relaxed seed JSONs available: " << relaxed_seed_jsons.size() << "\n";
  }

  // Heuristic seed build + save (+ include relaxed seeds in ranking)
  auto heuristic_ranked = build_and_save_heuristic_seeds(cfg, flags.verbose, tasks, matrix, relaxed_seed_jsons);

  // ALNS (stubbed until inspiration::run_alns is ready)
  if (flags.mode == "alns" || flags.mode == "all")
  {
    if (flags.verbose)
    {
      std::cout << "🧩 ALNS starting..."
                << " time=" << cfg.alns_time_limit_s << "s"
                << " moves=" << cfg.alns_moves
                << " target_veh=" << cfg.target_vehicles << "\n";
    }

    json final_result;
    try
    {
      // Wire this when your ALNS is ready:
      // final_result = inspiration::run_alns(tasks, matrix, cfg.base, relaxed_seed_jsons, alns_params);
      final_result = {{"meta", {{"note", "ALNS not yet implemented"}}}};
    }
    catch (const std::exception &e)
    {
      std::cerr << "ALNS failed: " << e.what() << "\n";
      return 3;
    }

    try
    {
      save_json(cfg.result_out, final_result);
    }
    catch (const std::exception &e)
    {
      std::cerr << "Failed to write result: " << e.what() << "\n";
      return 4;
    }

    if (flags.verbose)
      std::cout << "✅ Result written to " << cfg.result_out << "\n";
  }

  if (flags.verbose)
    std::cout << "🏁 Done.\n";
  return 0;
}
