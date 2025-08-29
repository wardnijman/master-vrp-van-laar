// first_schedule_middleman.h
#pragma once
#include "third_party/json/single_include/nlohmann/json.hpp"
#include <vector>
#include <string>

namespace fsmm { // first-schedule middleman

using json = nlohmann::json;

struct Config {
  int runs = 24;                    // independent relaxed runs at 26 vehicles
  int vehicles_relaxed = 26;        // relaxed stage
  int vehicles_target  = 25;        // final stage
  int time_limit_relaxed_s = 30;    // per relaxed run
  int time_limit_polish_s  = 25;    // per 25-vehicle polish
  double diversity_threshold = 0.16; // assignment-based distance
  int random_seed_base = 12345;     // varied per run: base + i
  bool verbose = false;
};


// Summary of any solution we keep during the pipeline
struct SolSummary {
  json solution;                 // full JSON from solver (post-parse_solution)
  std::vector<std::vector<std::vector<std::string>>> routes_by_day; // [day][veh][task_id]
  std::string signature;         // stable assignment signature
  double travel_minutes = 0.0;
  double overtime_minutes = 0.0;
  int vehicles_per_day = 0;
  int days = 0;
  int source_seed = 0; 
  int avg_load_minutes = 0;
  int max_load_minutes = 0;          // RNG seed that produced it
};

// Top-level: produce distinct 25-vehicle seeds (polished) from tasks+matrix
// Returns an array of seed JSON solutions (each a full parse_solution JSON).
std::vector<json> build_seeds(const json& tasks,
                              const json& matrix_seconds,
                              const json& base_config,
                              const Config& mmcfg);

// Helper for final selection if you need it outside:
// Greedy pick by (overtime, travel) with diversity gate on assignment distance.
std::vector<int> select_diverse_indices(const std::vector<SolSummary>& sols,
                                        double distance_threshold);

// Compute distance between two solutions by assignment (no Hungarian):
// distance = 1 - (#tasks with identical (day,route_idx) in both) / (#tasks in intersection)
// Assumes both are complete (no dropped tasks).
double assignment_distance(const SolSummary& a, const SolSummary& b);

} // namespace fsmm
