#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_index_manager.h"

// Your project types:
#include "first_schedule_solver.h"  // for TaskNode (must include: task_id, day, client_id, routing_node or equivalent)

// ---------- Extraction (from a solved multi-day first-schedule model) ----------

    using namespace operations_research;
using NodeIndex = RoutingIndexManager::NodeIndex;

// Build day -> route -> ordered task_ids (depot omitted).
std::vector<std::vector<std::vector<std::string>>>
extract_routes_task_ids_from_firstschedule(
    const operations_research::Assignment* solution,
    const operations_research::RoutingModel& routing,
    const operations_research::RoutingIndexManager& manager,
    const std::vector<TaskNode>& task_nodes,
    int vehicles_per_day,
    int num_days);

// Build day -> route -> total (service+travel) minutes using a time dimension.
std::vector<std::vector<int>>
extract_route_minutes_from_firstschedule(
    const operations_research::Assignment* solution,
    const operations_research::RoutingModel& routing,
    const operations_research::RoutingIndexManager& manager,
    const operations_research::RoutingDimension& time_dim,
    int vehicles_per_day,
    int num_days);

// Build a lookup from (task_id, day_idx) to NodeIndex (manager-level node).
// Use manager.NodeToIndex(node_index) to get the RoutingIndex needed by ReadAssignmentFromRoutes.
std::unordered_map<std::string, int>
build_taskid_date_index(
    const std::vector<TaskNode>& task_nodes,
    int num_days);

// ---------- Warm start construction (for polishing at target vehicle count) ----------

// Build an Assignment* from routes_by_day (task_ids), using the (task_id|day_idx)->NodeIndex map.
// vehicles_per_day / num_days define the flattening order: vehicle v = day*vehicles_per_day + route_idx
// ignore_inactive is typically true.
std::unique_ptr<operations_research::Assignment>
read_assignment_from_routes_task_ids(
    const operations_research::RoutingModel& routing,
    const operations_research::RoutingIndexManager& manager,
    const std::vector<std::vector<std::vector<std::string>>>& routes_by_day,
    const std::unordered_map<std::string, int>& taskid_day_to_nodeindex,
    int vehicles_per_day,
    int num_days,
    bool ignore_inactive = true);

