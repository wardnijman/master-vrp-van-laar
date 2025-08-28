// solution_parser.cpp (minimal output for downstream seed-building)
#include "first_schedule_solver.h"
#include "utils.h"
#include "seed.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <cassert>
#include <regex>
#include <ortools/constraint_solver/routing.h>
#include <ortools/constraint_solver/routing_parameters.h>
#include "third_party/json/single_include/nlohmann/json.hpp"
#include <unordered_map>

using namespace operations_research;
using json = nlohmann::json;
using NodeIndex = RoutingIndexManager::NodeIndex;

const int TIME_UNIT = 60; // seconds to minutes

nlohmann::json build_routes_by_day_json(
    const RoutingModel &routing,
    const RoutingIndexManager &manager,
    const Assignment *solution,
    int vehicles_per_day,
    int num_days,
    const std::vector<TaskNode> &task_nodes)
{
    // Map var-index -> task_id (skip depot 0)
    std::unordered_map<int64_t, std::string> var_to_task;
    var_to_task.reserve(task_nodes.size());
    for (const auto &tn : task_nodes)
    {
        int64_t var_idx = manager.NodeToIndex(RoutingIndexManager::NodeIndex(tn.routing_node));
        var_to_task[var_idx] = tn.task_id;
    }
    // Prepare [day][veh][task_id...]
    nlohmann::json routes_by_day = nlohmann::json::array();

    for (int d = 0; d < num_days; ++d)
    {
        // each day is an array of vehicles
        nlohmann::json vehicles = nlohmann::json::array();
        for (int k = 0; k < vehicles_per_day; ++k)
        {
            // each vehicle starts as empty task_id array
            vehicles.push_back(nlohmann::json::array());
        }
        routes_by_day.push_back(std::move(vehicles));
    }

    // Traverse each global vehicle route and push task_ids into the right [day][veh]
    const int num_vehicles = routing.vehicles();
    for (int v = 0; v < num_vehicles; ++v)
    {
        int day = v / vehicles_per_day;
        int veh_in_day = v % vehicles_per_day;

        int64_t index = routing.Start(v);
        while (!routing.IsEnd(index))
        {
            int64_t next = solution->Value(routing.NextVar(index));

            // If this index is a real task, append its task_id
            auto it = var_to_task.find(index);
            if (it != var_to_task.end())
            {
                routes_by_day[day][veh_in_day].push_back(it->second);
            }

            index = next;
        }
    }

    return routes_by_day;
}


json parse_solution(
    const Assignment *solution,
    const RoutingModel &routing,
    const RoutingIndexManager &manager,
    const RoutingDimension & /*time_dim*/,
    const std::vector<TaskNode> &task_nodes,
    const std::unordered_map<std::string, TaskNode> &task_map,
    const std::map<std::pair<int, int>, std::map<char, std::vector<const TaskNode *>>> & /*schema_week_suffix_map*/,
    const std::vector<std::vector<int>> & /*raw_matrix*/,
    const std::string & /*min_date*/,
    const std::unordered_map<std::string, std::vector<nlohmann::json>> & /*task_day_options*/,
    int vehicles_per_day,
    int num_days,
    int num_vehicles)
{
    json out;
    if (!solution)
    {
        out["error"] = "No solution found";
        out["assignments"] = json::array();
        out["dropped"] = json::array();
        out["canonical_weekday_by_schema"] = json::object();
        return out;
    }

    // Map routing node -> TaskNode
    std::unordered_map<int, TaskNode> rnode_to_task;
    rnode_to_task.reserve(task_nodes.size());
    for (const auto &n : task_nodes)
        rnode_to_task[n.routing_node] = n;

    json assignments = json::array();
    std::vector<std::pair<std::string, std::string>> id_date_pairs;
    id_date_pairs.reserve(task_nodes.size());

    // Collect chosen (task_id, date)
    for (int v = 0; v < num_vehicles; ++v)
    {
        int idx = routing.Start(v);
        idx = solution->Value(routing.NextVar(idx));
        while (!routing.IsEnd(idx))
        {
            int node = manager.IndexToNode(idx).value();
            auto it = rnode_to_task.find(node);
            if (it != rnode_to_task.end())
            {
                const TaskNode &t = it->second;
                assignments.push_back({{"task_id", t.task_id},
                                       {"date", t.day}});
                id_date_pairs.emplace_back(t.task_id, t.day);
            }
            idx = solution->Value(routing.NextVar(idx));
        }
    }

    // Dropped disjunction alternatives (not selected)
    json dropped = json::array();
    for (const auto &n : task_nodes)
    {
        int ridx = manager.NodeToIndex(NodeIndex(n.routing_node));
        if (solution->Value(routing.NextVar(ridx)) == ridx)
        {
            dropped.push_back({{"task_id", n.task_id}, {"date", n.day}});
        }
    }

    // Build canonical weekday per schema_number (mode of assigned weekdays)
    // First: task_id -> schema_number (from task_map)
    std::unordered_map<std::string, int> task_to_schema;
    task_to_schema.reserve(task_map.size());
    for (const auto &kv : task_map)
        task_to_schema[kv.first] = kv.second.schema_number;

    std::unordered_map<int, std::array<int, 8>> counts; // 1..7
    for (auto &p : id_date_pairs)
    {
        auto it = task_to_schema.find(p.first);
        if (it == task_to_schema.end())
            continue;
        int wd = weekday1_from_date(p.second);
        if (wd < 1 || wd > 7)
            continue;
        auto &arr = counts[it->second];
        if (arr[0] == 0)
            arr.fill(0);
        arr[wd] += 1;
    }

    json canon_json = json::object();
    for (auto &kv : counts)
    {
        int best_w = 1, best_c = -1;
        for (int w = 1; w <= 7; ++w)
        {
            if (kv.second[w] > best_c)
            {
                best_c = kv.second[w];
                best_w = w;
            }
        }
        canon_json[std::to_string(kv.first)] = best_w;
    }

    out["routes_by_day"] = build_routes_by_day_json(
        routing, manager, solution,
        /*vehicles_per_day=*/vehicles_per_day,
        /*num_days=*/num_days,
        /*task_nodes=*/task_nodes);

    out["assignments"] = std::move(assignments);
    out["dropped"] = std::move(dropped);
    out["canonical_weekday_by_schema"] = std::move(canon_json);
    return out;
}

