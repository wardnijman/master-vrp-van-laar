#include "day_solver.h"
#include <memory> // at top if missing

#include <unordered_map>
#include <utility>
#include <iostream>

#include <ortools/constraint_solver/routing.h>
#include <ortools/constraint_solver/routing_parameters.h>

using operations_research::Assignment;
using operations_research::RoutingIndexManager;
using operations_research::RoutingModel;
using operations_research::RoutingSearchParameters;

namespace
{

    // Build global->sub mapping for the current subproblem.
    // sub indexing convention: 0 = depot(0), 1..N = day clients in order.
    static std::unordered_map<int, int> make_g2s(const std::vector<int> &sub_to_global)
    {
        std::unordered_map<int, int> g2s;
        g2s.reserve(sub_to_global.size());
        for (int sub = 0; sub < (int)sub_to_global.size(); ++sub)
        {
            g2s.emplace(sub_to_global[sub], sub);
        }
        return g2s;
    }

    // Convert warm-start routes expressed as GLOBAL client IDs (no depots)
    // into subproblem indices expected by ReadAssignmentFromRoutes.
    // - Drops any client not present today;
    // - Ensures no depot (0) entries are included;
    // - Resizes to number of vehicles by adding empty routes if needed.
    static std::vector<std::vector<int64_t>>
    build_routes_sub_from_warm(const std::vector<std::vector<int>> &warm_routes_global,
                               const std::unordered_map<int, int> &g2s,
                               int vehicles)
    {
        std::vector<std::vector<int64_t>> routes_sub;
        routes_sub.reserve(vehicles);

        // Copy/convert up to 'vehicles' routes; if fewer, pad with empties.
        const int copy_count = std::min<int>((int)warm_routes_global.size(), vehicles);
        for (int r = 0; r < copy_count; ++r)
        {
            const auto &route_g = warm_routes_global[r];
            std::vector<int64_t> route_s;
            route_s.reserve(route_g.size());
            for (int gid : route_g)
            {
                if (gid == 0)
                    continue; // never include depot in ReadAssignmentFromRoutes
                auto it = g2s.find(gid);
                if (it == g2s.end())
                    continue; // client not in today's set -> drop
                const int sub = it->second;
                if (sub == 0)
                    continue; // safety: skip depot if somehow mapped
                route_s.push_back(static_cast<int64_t>(sub));
            }
            routes_sub.push_back(std::move(route_s));
        }
        for (int r = copy_count; r < vehicles; ++r)
        {
            routes_sub.emplace_back(); // empty route
        }
        return routes_sub;
    }

} // namespace

DayResult solve_day(const std::vector<TaskRef> &day_tasks,
                    const TimeMatrix &M,
                    const DayConfig &cfg,
                    const DaySolveParams &params,
                    const DayWarmStart *warm)
{
    DayResult out;

    // ---- Build compact subproblem mapping ----
    std::vector<int> sub_to_global;
    sub_to_global.reserve(day_tasks.size() + 1);
    sub_to_global.push_back(0); // depot
    for (const auto &t : day_tasks)
        sub_to_global.push_back(t.client_idx);
    const int N = static_cast<int>(sub_to_global.size());

    // Service time per sub-node (minutes)
    std::vector<int> sub_service(N, 0);
    for (int i = 1; i < N; ++i)
        sub_service[i] = day_tasks[i - 1].service_minutes;

    // ---- PRECOMPUTE submatrix: travel + service(from) ----
    std::vector<int> sub_cost;
    sub_cost.resize(N * N);
    for (int i = 0; i < N; ++i)
    {
        const int g_from = sub_to_global[i];
        const int service = sub_service[i];
        int base = i * N;
        for (int j = 0; j < N; ++j)
        {
            const int g_to = sub_to_global[j];
            const int travel = M.at(g_from, g_to); // minutes
            sub_cost[base + j] = travel + service;
        }
    }

    const int vehicles = cfg.vehicles;

    // ---- OR-Tools model ----
    RoutingIndexManager manager(
        /*num_nodes=*/N,
        /*num_vehicles=*/vehicles,
        /*depot=*/RoutingIndexManager::NodeIndex(0));
    RoutingModel routing(manager);

    // Transit callback: O(1) lookup in precomputed table
    const int transit_cb = routing.RegisterTransitCallback(
        [&manager, &sub_cost, N](int64_t from_index, int64_t to_index) -> int64_t
        {
            const int from_sub = manager.IndexToNode(from_index).value();
            const int to_sub = manager.IndexToNode(to_index).value();
            return static_cast<int64_t>(sub_cost[from_sub * N + to_sub]);
        });

    routing.SetArcCostEvaluatorOfAllVehicles(transit_cb);

    // ---- Time dimension: soft horizon via soft UB on end cumul ----
    const int64_t soft_horizon = cfg.route_limit_minutes; // e.g., 540
    const int64_t big_cap = 1000000;                      // large enough to not bind

    routing.AddDimension(/*transit=*/transit_cb,
                         /*slack_max=*/0,
                         /*capacity=*/big_cap,
                         /*fix_start_cumul_to_zero=*/true,
                         /*name=*/"Time");
    auto *time_dim = routing.GetMutableDimension("Time");

    // Soft upper bound at route end for each vehicle
    for (int v = 0; v < vehicles; ++v)
    {
        time_dim->SetCumulVarSoftUpperBound(routing.End(v), soft_horizon, cfg.soft_upper_cost);
    }

    // Encourage compact routes
    time_dim->SetSpanCostCoefficientForAllVehicles(cfg.span_coeff);

    // ---- Search parameters ----
    RoutingSearchParameters p = operations_research::DefaultRoutingSearchParameters();
    p.set_first_solution_strategy(operations_research::FirstSolutionStrategy::PATH_CHEAPEST_ARC);
    p.set_local_search_metaheuristic(operations_research::LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
    p.set_log_search(params.log_search);
    if (params.time_limit_seconds > 0)
        p.mutable_lns_time_limit()->set_seconds(params.time_limit_seconds);
    p.mutable_time_limit()->set_seconds(params.time_limit_seconds);
    p.set_number_of_solutions_to_collect(1);

    auto *ops = p.mutable_local_search_operators();
    using operations_research::BOOL_TRUE;
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

    // ---- Try warm start if requested ----
    const Assignment *assignment = nullptr;
    if (params.use_warm_start && warm && !warm->routes_global.empty())
    {
        // Build g2s map for today's subproblem
        const auto g2s = make_g2s(sub_to_global);

        // Convert global→sub and pad/truncate to vehicle count
        auto routes_sub = build_routes_sub_from_warm(warm->routes_global, g2s, vehicles);

        // Read seed assignment
        std::unique_ptr<operations_research::Assignment> seed(
            routing.ReadAssignmentFromRoutes(routes_sub, /*ignore_inactive=*/true));
        if (seed)
        {
            // expects const Assignment*
            assignment = routing.SolveFromAssignmentWithParameters(seed.get(), p);
        }
        // If seed rejected or leads to no solution within limits, we fall back below.
    }

    // ---- Solve cold if no warm or warm failed ----
    if (!assignment)
    {
        assignment = routing.SolveWithParameters(p);
    }

    if (!assignment)
    {
        if (params.log_search)
        {
            std::cerr << "[day_solver] infeasible day: N=" << (N - 1)
                      << " vehicles=" << vehicles
                      << " soft_horizon=" << cfg.route_limit_minutes << " min\n";
        }
        out.feasible = false;
        return out;
    }

    // ---- Extract solution ----
    int total_minutes = 0, used = 0, overtime_sum = 0;

    for (int v = 0; v < vehicles; ++v)
    {
        int64_t idx = routing.Start(v);
        if (routing.IsEnd(assignment->Value(routing.NextVar(idx))))
            continue;

        used++;
        std::vector<int> seq; // global indices, include depot at start/end for continuity
        while (!routing.IsEnd(idx))
        {
            const int sub = manager.IndexToNode(idx).value();
            seq.push_back(sub_to_global[sub]);
            idx = assignment->Value(routing.NextVar(idx));
        }
        seq.push_back(0); // return to depot

        const int route_min = static_cast<int>(assignment->Value(time_dim->CumulVar(routing.End(v))));
        total_minutes += route_min;
        if (route_min > cfg.route_limit_minutes)
            overtime_sum += (route_min - cfg.route_limit_minutes);

        out.routes.push_back({std::move(seq), route_min});
    }

    out.used_vehicles = used;
    out.overtime_minutes = overtime_sum;
    out.total_minutes = total_minutes;
    out.feasible = true;

    return out;
}
