#pragma once
#include <vector>
#include <string>
#include "third_party/json/single_include/nlohmann/json.hpp"
#include <ortools/constraint_solver/routing.h>
#include <ortools/constraint_solver/routing_parameters.h>

using namespace operations_research;

namespace first_schedule_solver
{
    using json = nlohmann::json;

    // Optional runtime options (all fields optional; negatives/empty = use defaults)
    // Add/extend your options struct
    struct SolveOptions
    {
        int vehicles_per_day = 0;
        int time_limit_seconds = 0;
        int lns_time_limit_seconds = 0;
        int num_solutions_to_collect = 1;
        bool log_search = false;

        // existing field you already pass from middleman; reuse for reproducibility
        int random_seed = 0;

        // NEW: deterministic diversity knobs
        // a) shuffle task nodes order before building the model
        bool shuffle_task_nodes = true;
        // b) seed for shuffle; if 0, fall back to random_seed
        int shuffle_seed = 0;

        // c) add small integer jitter to arc costs only (minutes)
        int cost_noise_magnitude = 0; // e.g., 0..2 minutes; 0 = off
        int cost_noise_seed = 0;      // if 0, fall back to random_seed

        // d) variety knob for first solution strategy
        //    (default stays PATH_CHEAPEST_ARC)
        operations_research::FirstSolutionStrategy::Value first_solution_strategy =
            operations_research::FirstSolutionStrategy::PATH_CHEAPEST_ARC;

        bool enable_operator_bundle = true; // turn on the big move set (like your old solver)

        // Warm-start: [days][vehicles_per_day][task_id]
        std::vector<std::vector<std::vector<std::string>>> warm_routes_by_day;

        // Allow small overtime during the solve (per route, minutes beyond MAX_HOURS_PER_DAY)
        int overtime_cap_minutes = 0; // 0 = strict cap (today’s behavior)
        int overtime_soft_coeff = 0;  // optional linear penalty per minute beyond the hard day limit
    };

    // Existing entry point (kept as-is)
    nlohmann::json solve_problem(const json &tasks,
                                 const json &dist_matrix_raw,
                                 const json &config);

    // New overload: configurable + optional warm start
    nlohmann::json solve_problem(const json &tasks,
                                 const json &dist_matrix_raw,
                                 const json &config,
                                 const SolveOptions &opts);

} // namespace first_schedule_solver

struct TaskNode
{
    int routing_node;
    std::string task_id;
    int schema_number;
    int client_id;
    int duration;
    int start;
    int end;
    std::string day; // <-- use 'day' (not 'date')
    std::vector<int> allowed_vehicles;
    int frequency;
};
