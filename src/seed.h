#pragma once
#include <string>
#include <vector>
#include <functional>
#include <unordered_map>
#include <optional>
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <fstream>

#include "types.h"
#include "day_solver.h"
#include "third_party/json/single_include/nlohmann/json.hpp"

// Forward decls – adapt to your real types if you already have them.
struct Task {
  std::string task_id;
  int schema_number = 0;
  int client_id = 0;
  double duration = 0.0;          // minutes
  int week = 0;

  struct DayOption {               // you said: always one day_option in seeds
    std::string date;              // "YYYY-MM-DD"
    int weekday = 0;               // 1..7? (adapt to your convention)
  };
  std::vector<DayOption> day_options; // in seeds: size == 1
  // ... other fields from your tasks.json (address, location, etc.)
};

struct Seed {
  std::string name;                // generator name or instance label
  std::vector<Task> tasks;         // full tasks list with exactly 1 day_option per task
};

struct SeedScore {
  double total_cost = 0.0;         // your unified score (travel + penalties)
  bool valid = false;              // passed validation
  std::string validation_error;    // empty if valid
};

using SeedGeneratorFn = std::function<Seed(const std::vector<Task>& base_tasks)>;

// Registry (singleton-style accessors).
struct SeedRegistry {
  static void register_generator(const std::string& name, SeedGeneratorFn fn);
  static bool has(const std::string& name);
  static SeedGeneratorFn get(const std::string& name);
  static std::vector<std::string> names();

private:
  static std::unordered_map<std::string, SeedGeneratorFn>& map();
};

// Scoring/evaluation hook you will wire to your day-VRP.
// Implemented in seed_scoring.cpp; you only need to fill in the callback body.
struct EvalConfig {
  // Existing:
  int max_routes_per_day = 25;
  int max_hours_per_route = 9;

  // New (required to run the real day solver):
  const TimeMatrix* matrix = nullptr;  // pointer to your global matrix (minutes)
  DayConfig day_cfg{};                 // vehicles, route_limit_minutes, soft_upper_cost, span_coeff
  DaySolveParams explore_params{};     // time limit, use_warm_start, log_search, etc.
};

SeedScore evaluate_seed_cost(const Seed& seed, const EvalConfig& cfg);

struct RankedSeed {
  Seed seed;
  SeedScore score;
};

// Evaluate a list of generator names on the same base task list; keep top-k.
std::vector<RankedSeed> build_and_rank_seeds(
  const std::vector<std::string>& generator_names,
  const std::vector<Task>& base_tasks,
  const EvalConfig& cfg,
  int top_k);

// Utility: stable sort by score (ascending).
void sort_by_cost(std::vector<RankedSeed>& v);


// Small helper to pretty print a ranked list
static void print_ranked_seeds(const std::vector<RankedSeed>& ranked) {
  std::cout << "\n# Seed ranking\n";
  std::cout << std::left << std::setw(24) << "name"
            << std::right << std::setw(16) << "cost"
            << std::setw(10) << "valid"
            << "   " << "note" << "\n";
  for (const auto& r : ranked) {
    std::cout << std::left << std::setw(24) << r.seed.name
              << std::right << std::setw(16) << std::fixed << std::setprecision(2) << r.score.total_cost
              << std::setw(10) << (r.score.valid ? "yes" : "no")
              << "   " << (r.score.validation_error.empty() ? "" : r.score.validation_error)
              << "\n";
  }
  std::cout << std::endl;
}



using json = nlohmann::json;
namespace fs = std::filesystem;

// ---- Seed JSON I/O ----

static json seed_task_to_json(const Task &t)
{
    // Writes back in the same “seed-shaped” format you described: 1 day_option
    json j;
    j["task_id"] = t.task_id;
    j["schema_number"] = t.schema_number;
    j["client_id"] = t.client_id;
    j["duration"] = t.duration; // minutes
    j["week"] = t.week;
    if (!t.day_options.empty())
    {
        j["day_options"] = json::array();
        j["day_options"].push_back({{"date", t.day_options.front().date},
                                    {"weekday", t.day_options.front().weekday}});
    }
    return j;
}

static void write_seed_json(const RankedSeed& rs, const fs::path& path) {
    json out = json::array();
    // Pre-reserve capacity on the underlying array container (if available)
    auto* arr = out.is_array() ? &out.get_ref<json::array_t&>() : nullptr;
    if (arr) arr->reserve(rs.seed.tasks.size());

    for (const auto& t : rs.seed.tasks) {
        out.push_back(seed_task_to_json(t));
    }

    fs::create_directories(path.parent_path());
    std::ofstream f(path);
    if (!f) throw std::runtime_error("Cannot open " + path.string() + " for writing.");
    f << out.dump(2) << "\n";
}

static std::vector<Task> load_seed_shaped_tasks_from_json(const json &tasks_json)
{
    if (!tasks_json.is_array())
        throw std::runtime_error("seed_tasks.json must be an array.");
    std::vector<Task> tasks;
    tasks.reserve(tasks_json.size());
    for (const auto &t : tasks_json)
    {
        Task x;
        x.task_id = t.at("task_id").get<std::string>();
        x.schema_number = t.at("schema_number").get<int>();
        x.client_id = t.at("client_id").get<int>();
        x.duration = (t.contains("duration_seconds"))
                         ? std::llround(t.at("duration_seconds").get<double>() / 60.0)
                         : t.at("duration").get<int>();
        x.week = t.at("week").get<int>();
        // Expect exactly one day_option; if multiple/none, we still accept and pick the first/leave empty.
        if (t.contains("day_options") && t["day_options"].is_array() && !t["day_options"].empty())
        {
            auto &opt = t["day_options"].front();
            Task::DayOption d;
            d.date = opt.value("date", "");
            d.weekday = opt.value("weekday", 0);
            x.day_options.push_back(std::move(d));
        }
        tasks.push_back(std::move(x));
    }
    return tasks;
}

// Validation used by main (exposed)
std::string run_validation(const std::vector<Task>& tasks);

// Build many candidates, keep best per generator by cost, then pick a diverse subset.
std::vector<RankedSeed> build_rank_and_pick_diverse(
  const std::vector<std::string>& generator_names,
  const std::vector<Task>& base_tasks,
  const EvalConfig& cfg,
  int per_generator_trials,
  int keep_top_by_cost_per_gen,
  int diverse_k,
  int rng_seed = 12345);