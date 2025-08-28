#pragma once
#include <string>
#include <vector>
#include <functional>
#include <unordered_map>
#include <optional>
#include "types.h"
#include "day_solver.h"

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
