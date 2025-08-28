#include "first_schedule_solver.h"
#include <ortools/constraint_solver/routing.h>
#include <ortools/constraint_solver/routing_parameters.h>
#include "first_solution_parser.h"
#include "third_party/json/single_include/nlohmann/json.hpp"

#include <iostream>
#include <fstream>
#include <cmath>
#include <unordered_map>
#include <map>
#include <set>
#include <cassert>
#include <thread>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <regex>
#include <algorithm>
#include <random>

namespace first_schedule_solver
{
    using namespace operations_research;
    using json = nlohmann::json;
    using NodeIndex = RoutingIndexManager::NodeIndex;

    // small splitmix64 for deterministic hashing/jitter
    static inline uint64_t splitmix64(uint64_t x)
    {
        x += 0x9e3779b97f4a7c15ULL;
        x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
        x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
        return x ^ (x >> 31);
    }

    const int TIME_UNIT = 60;           // seconds to minutes
    const int DEFAULT_MAX_RUNTIME = 60; // seconds

    // yyyy-mm-dd helpers
    int date_diff_days(const std::string &start, const std::string &end)
    {
        std::tm tm_start = {}, tm_end = {};
        std::istringstream ss_start(start);
        std::istringstream ss_end(end);
        ss_start >> std::get_time(&tm_start, "%Y-%m-%d");
        ss_end >> std::get_time(&tm_end, "%Y-%m-%d");

        auto t_start = std::chrono::system_clock::from_time_t(std::mktime(&tm_start));
        auto t_end = std::chrono::system_clock::from_time_t(std::mktime(&tm_end));
        return static_cast<int>(std::chrono::duration_cast<std::chrono::hours>(t_end - t_start).count() / 24) + 1;
    }

    std::string add_days(const std::string &start_date, int offset_days)
    {
        std::tm tm = {};
        std::istringstream ss(start_date);
        ss >> std::get_time(&tm, "%Y-%m-%d");
        auto time = std::chrono::system_clock::from_time_t(std::mktime(&tm));
        time += std::chrono::hours(24 * offset_days);
        std::time_t new_time = std::chrono::system_clock::to_time_t(time);
        std::ostringstream out;
        out << std::put_time(std::localtime(&new_time), "%Y-%m-%d");
        return out.str();
    }

    // OLD entry point now forwards to new overload with defaults
    json solve_problem(const json &tasks, const json &dist_matrix_raw, const json &config)
    {
        SolveOptions def{};
        return solve_problem(tasks, dist_matrix_raw, config, def);
    }

    json solve_problem(const json &tasks, const json &dist_matrix_raw, const json &config, const SolveOptions &opts)
    {
        // Pull core params with overrides
        const int max_hours = config.value("MAX_HOURS_PER_DAY", 9);
        const int vehicles_per_day = (opts.vehicles_per_day > 0)
                                         ? opts.vehicles_per_day
                                         : config.value("VEHICLES_PER_DAY", 25);
        const int max_duration = max_hours * 3600 / TIME_UNIT;

        const int time_limit_s = (opts.time_limit_seconds > 0)
                                     ? opts.time_limit_seconds
                                     : config.value("TIME_LIMIT_SECONDS", DEFAULT_MAX_RUNTIME);

        // matrix
        std::vector<std::vector<int>> raw_matrix;
        raw_matrix.reserve(dist_matrix_raw.size());
        for (const auto &row : dist_matrix_raw)
            raw_matrix.push_back(row.get<std::vector<int>>());

        // Build task_nodes
        std::vector<TaskNode> task_nodes;
        std::vector<std::string> all_dates;
        all_dates.reserve(tasks.size() * 2);

        for (const auto &task : tasks)
        {
            for (const auto &opt : task["day_options"])
                all_dates.push_back(opt["date"]);
        }
        std::sort(all_dates.begin(), all_dates.end());
        all_dates.erase(std::unique(all_dates.begin(), all_dates.end()), all_dates.end());

        std::string min_date = all_dates.front();
        std::string max_date = all_dates.back();
        int num_days = date_diff_days(min_date, max_date);
        int num_vehicles = vehicles_per_day * num_days;

        int routing_index = 1;
        task_nodes.reserve(tasks.size() * 2);
        for (const auto &task : tasks)
        {
            int client_id = task["client_id"];
            int schema_number = task["schema_number"];
            std::string task_id = task["task_id"];
            int duration = std::max(1, (int)task["duration"]);
            int frequency = task["frequency"]["value"];

            for (const auto &opt : task["day_options"])
            {
                std::string date = opt["date"];
                int day_index = date_diff_days(min_date, date) - 1;
                int start = day_index * max_duration;
                int end = start + max_duration;

                std::vector<int> vehicles;
                vehicles.reserve(vehicles_per_day);
                for (int i = 0; i < vehicles_per_day; ++i)
                    vehicles.push_back(day_index * vehicles_per_day + i);

                task_nodes.push_back({routing_index++, task_id, schema_number, client_id,
                                      duration, start, end, date, vehicles, frequency});
            }
        }

        if (opts.shuffle_task_nodes)
        {
            int seed = opts.shuffle_seed ? opts.shuffle_seed : opts.random_seed;
            if (seed != 0)
            {
                std::mt19937 rng(seed);
                std::shuffle(task_nodes.begin(), task_nodes.end(), rng);
                // routing_node ids must be 1..N in the new order
                for (int i = 0; i < (int)task_nodes.size(); ++i)
                    task_nodes[i].routing_node = i + 1;
            }
        }

        // Maps for disjunctions, parsing, and warm-start lookup
        std::unordered_map<std::string, std::vector<int>> disjunction_groups_map; // index list per task_id
        std::unordered_map<std::string, TaskNode> task_map_single;                // last seen (used by parser)
        // NEW: task_id + day_index -> routing_node for warm-start
        std::unordered_map<std::string, std::unordered_map<int, int>> task_day_to_node;

        for (const auto &node : task_nodes)
        {
            int day_idx = date_diff_days(min_date, node.day) - 1; // FIX: use node.day
            int index = node.routing_node;
            disjunction_groups_map[node.task_id].push_back(index);
            task_map_single[node.task_id] = node;
            task_day_to_node[node.task_id][day_idx] = node.routing_node;
        }

        // schema_week_suffix_map
        std::map<std::pair<int, int>, std::map<char, std::vector<const TaskNode *>>> schema_week_suffix_map;
        std::regex re(R"(^(\d+)-W(\d+)(?:-([a-z]))?$)");
        for (const auto &node : task_nodes)
        {
            std::smatch m;
            if (!std::regex_match(node.task_id, m, re))
                continue;
            int schema = node.schema_number;
            int week = std::stoi(m[2]);
            char suffix = (m[3].matched ? m[3].str()[0] : '\0');
            schema_week_suffix_map[{schema, week}][suffix].push_back(&node);
        }

        int n = (int)task_nodes.size() + 1;
        NodeIndex depot(0);
        std::vector<NodeIndex> starts(num_vehicles, depot), ends(num_vehicles, depot);
        RoutingIndexManager manager(n, num_vehicles, starts, ends);
        RoutingModel routing(manager);

        // durations
        std::unordered_map<int, int> durations;
        durations[0] = 0;
        for (const auto &node : task_nodes)
            durations[node.routing_node] = node.duration;

        const int cost_noise_mag = std::max(0, opts.cost_noise_magnitude);
        const int cost_noise_seed = (opts.cost_noise_seed != 0) ? opts.cost_noise_seed : opts.random_seed;

        const auto distance_cb = [&manager, &task_nodes, &raw_matrix, cost_noise_mag, cost_noise_seed](int64_t from, int64_t to) -> int64_t
        {
            int from_node = manager.IndexToNode(from).value();
            int to_node = manager.IndexToNode(to).value();
            int from_id = (from_node == 0) ? 0 : task_nodes[from_node - 1].client_id;
            int to_id = (to_node == 0) ? 0 : task_nodes[to_node - 1].client_id;

            int base = raw_matrix[from_id][to_id] / TIME_UNIT;

            // if (cost_noise_mag > 0)
            // {
            //     // deterministic tiny jitter in minutes
            //     uint64_t key = ((uint64_t)from_id << 32) ^ (uint64_t)to_id ^ (uint64_t)cost_noise_seed;
            //     int jitter = (int)(splitmix64(key) % (uint64_t)(cost_noise_mag + 1));
            //     base += jitter;
            // }
            return base;
        };
        int arc_cost_cb = routing.RegisterTransitCallback(distance_cb);
        routing.SetArcCostEvaluatorOfAllVehicles(arc_cost_cb);

        const auto time_cb = [&manager, &task_nodes, &raw_matrix, &durations](int64_t from, int64_t to)
        {
            int from_node = manager.IndexToNode(from).value();
            int to_node = manager.IndexToNode(to).value();
            int from_id = (from_node == 0) ? 0 : task_nodes[from_node - 1].client_id;
            int to_id = (to_node == 0) ? 0 : task_nodes[to_node - 1].client_id;
            int travel_time = raw_matrix[from_id][to_id] / TIME_UNIT;
            return travel_time + durations.at(from_node);
        };
        int time_cb_idx = routing.RegisterTransitCallback(time_cb);

        // // minutes per day
        // const int route_limit = max_duration; // already in minutes
        // const int ot_cap = std::max(0, opts.overtime_cap_minutes);

        // // The global horizon must allow the extra minutes.
        // const int horizon = (route_limit + ot_cap) * num_days;

        // routing.AddDimensionWithVehicleTransits(
        //     std::vector<int>(num_vehicles, time_cb_idx),
        //     /*slack*/ 0,
        //     /*capacity*/ horizon,
        //     /*fix_start_cumul_to_zero*/ false,
        //     "Time");

        // RoutingDimension *time_dim_mut =
        //     const_cast<RoutingDimension *>(&routing.GetDimensionOrDie("Time"));
        // const RoutingDimension &time_dim = *time_dim_mut;

        // // Allowed vehicles
        // for (const auto &node : task_nodes)
        // {
        //     int index = manager.NodeToIndex(NodeIndex(node.routing_node));
        //     routing.SetAllowedVehiclesForIndex(node.allowed_vehicles, index);
        // }

        // // Per-vehicle day ranges
        // for (int v = 0; v < num_vehicles; ++v)
        // {
        //     int day = v / vehicles_per_day;
        //     int start_idx = routing.Start(v);
        //     int end_idx = routing.End(v);

        //     const int day_start = day * route_limit; // minutes
        //     const int hard_day_end = (day + 1) * route_limit;
        //     const int hard_vehicle_end = hard_day_end + ot_cap; // allow +ot_cap in polish

        //     time_dim.CumulVar(start_idx)->SetRange(day_start, hard_day_end);
        //     time_dim.CumulVar(end_idx)->SetRange(day_start, hard_vehicle_end);

        //     // Optional: soft penalty for spilling beyond the true day cap.
        //     if (opts.overtime_soft_coeff > 0)
        //     {
        //         time_dim_mut->SetCumulVarSoftUpperBound(end_idx, hard_day_end,
        //                                                 opts.overtime_soft_coeff);
        //     }
        // }

        // minutes per day
        const int route_limit = max_duration; // minutes
        const int ot_cap = std::max(0, opts.overtime_cap_minutes);

        // Global horizon (only add OT minutes if we actually allow OT).
        const int horizon = (ot_cap > 0 ? (route_limit + ot_cap) : route_limit) * num_days;

        routing.AddDimensionWithVehicleTransits(
            std::vector<int>(num_vehicles, time_cb_idx),
            /*slack*/ 0,
            /*capacity*/ horizon,
            /*fix_start_cumul_to_zero*/ false,
            "Time");

        RoutingDimension *time_dim_mut =
            const_cast<RoutingDimension *>(&routing.GetDimensionOrDie("Time"));
        const RoutingDimension &time_dim = *time_dim_mut;

        // Per-vehicle day windows
        for (int v = 0; v < num_vehicles; ++v)
        {
            const int day = v / vehicles_per_day;
            const int start_idx = routing.Start(v);
            const int end_idx = routing.End(v);

            const int day_start = day * route_limit;
            const int hard_day_end = (day + 1) * route_limit;
            const int hard_vehicle_end = (ot_cap > 0) ? (hard_day_end + ot_cap) : hard_day_end;

            time_dim.CumulVar(start_idx)->SetRange(day_start, hard_day_end);
            time_dim.CumulVar(end_idx)->SetRange(day_start, hard_vehicle_end);

            // Only add a soft-UB penalty if user explicitly asked for it (>0).
            if (opts.overtime_soft_coeff > 0)
            {
                time_dim_mut->SetCumulVarSoftUpperBound(end_idx, hard_day_end, opts.overtime_soft_coeff);
            }
        }

        // Disjunctions (at-most-one day-option per task)
        const int penalty = 1000000; // external logic should reject drops; high soft penalty
        for (const auto &kv : disjunction_groups_map)
        {
            const auto &group = kv.second;
            std::vector<int64_t> group64(group.begin(), group.end());
            routing.AddDisjunction(group64, penalty);
        }

        // Search params with overrides
        RoutingSearchParameters search_params = DefaultRoutingSearchParameters();
        search_params.set_first_solution_strategy(opts.first_solution_strategy);
        // keep the rest as you have it:
        search_params.set_local_search_metaheuristic(LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
        search_params.set_number_of_solutions_to_collect(std::max(1, opts.num_solutions_to_collect));
        search_params.set_log_search(opts.log_search);
        search_params.mutable_time_limit()->set_seconds(time_limit_s);
        search_params.mutable_lns_time_limit()->set_seconds(std::max(0, opts.lns_time_limit_seconds));

        // NOTE: Some OR-Tools builds don't expose random_seed on RoutingSearchParameters.
        // If yours does, you can uncomment the next line.
        // search_params.set_use_randomization(true);
        // if (opts.random_seed > 0) search_params.set_random_seed(opts.random_seed);

        // Bring back the strong operator bundle (matches your old solver.cpp)
        if (opts.enable_operator_bundle)
        {
            using operations_research::BOOL_TRUE;
            using operations_research::OptionalBoolean;
            auto *ops = search_params.mutable_local_search_operators();

            ops->set_use_make_active(BOOL_TRUE);
            ops->set_use_make_inactive(BOOL_TRUE);
            ops->set_use_swap_active(BOOL_TRUE);
            ops->set_use_extended_swap_active(BOOL_TRUE);
            ops->set_use_relocate_and_make_active(BOOL_TRUE);

            ops->set_use_relocate(BOOL_TRUE);
            ops->set_use_two_opt(BOOL_TRUE);
            ops->set_use_or_opt(BOOL_TRUE);
            ops->set_use_exchange(BOOL_TRUE);
            ops->set_use_cross(BOOL_TRUE);

            ops->set_use_path_lns(BOOL_TRUE);
            ops->set_use_inactive_lns(BOOL_TRUE);
        }

        // Solve (fresh or warm)
        const Assignment *solution = nullptr;
        if (!opts.warm_routes_by_day.empty())
        {
            // Build var-index routes from warm task_ids
            if ((int)opts.warm_routes_by_day.size() != num_days)
                throw std::runtime_error("warm_routes_by_day: num_days mismatch");

            std::vector<std::vector<int64_t>> var_routes(num_vehicles);
            for (int d = 0; d < num_days; ++d)
            {
                const auto &veh_vec = opts.warm_routes_by_day[d];
                if ((int)veh_vec.size() != vehicles_per_day)
                    throw std::runtime_error("warm_routes_by_day: vehicles_per_day mismatch at day " + std::to_string(d));

                for (int k = 0; k < vehicles_per_day; ++k)
                {
                    int v = d * vehicles_per_day + k; // global vehicle index
                    for (const auto &task_id : veh_vec[k])
                    {
                        auto it_day = task_day_to_node.find(task_id);
                        if (it_day == task_day_to_node.end() || !it_day->second.count(d))
                            throw std::runtime_error("warm start refers to task_id/day not present: " + task_id + " day " + std::to_string(d));
                        int node_idx = it_day->second.at(d); // routing node index (1..n-1)
                        int64_t var_idx = manager.NodeToIndex(NodeIndex(node_idx));
                        var_routes[v].push_back(var_idx);
                    }
                }
            }
            const Assignment *seed = routing.ReadAssignmentFromRoutes(var_routes, /*ignore_inactive_nodes=*/true);
            solution = routing.SolveFromAssignmentWithParameters(seed, search_params); // FIX: pass pointer
        }
        else
        {
            solution = routing.SolveWithParameters(search_params);
        }

        // Parse JSON
        std::unordered_map<std::string, std::vector<nlohmann::json>> task_day_options;
        for (const auto &task : tasks)
            task_day_options[task["task_id"]] = task["day_options"].get<std::vector<nlohmann::json>>();

        // Parse + now also get routes_by_day embedded
        return parse_solution(
            solution,
            routing,
            manager,
            time_dim,
            task_nodes,
            task_map_single,
            schema_week_suffix_map,
            raw_matrix,
            min_date,
            task_day_options,
            vehicles_per_day,
            num_days,
            num_vehicles);
    }

} // namespace first_schedule_solver
