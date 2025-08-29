#include "routes_extract.h"

#include <algorithm>
#include <cassert>
#include <sstream>

// Key helper: build a stable key for (task_id, day_idx)
static inline std::string key_tid_day(const std::string& tid, int day_idx) {
  std::string k;
  k.reserve(tid.size() + 8);
  k.append(tid);
  k.push_back('|');
  k.append(std::to_string(day_idx));
  return k;
}

// Build maps from OR-Tools nodes → TaskNode and day string → day_idx
static void build_node_and_day_maps(
    const std::vector<TaskNode>& task_nodes,
    std::unordered_map<int, const TaskNode*>& node_to_task,
    std::unordered_map<std::string, int>& day_to_idx,
    std::vector<std::string>& idx_to_day)
{
  node_to_task.clear();
  day_to_idx.clear();
  idx_to_day.clear();

  // Days (sorted)
  {
    std::vector<std::string> days;
    days.reserve(task_nodes.size() / 16 + 8);
    for (const auto& tn : task_nodes) days.push_back(tn.day);
    std::sort(days.begin(), days.end());
    days.erase(std::unique(days.begin(), days.end()), days.end());
    idx_to_day = days;
    for (int i = 0; i < (int)days.size(); ++i) day_to_idx[days[i]] = i;
  }

  // Map routing_node (manager-level node) → TaskNode*
  // Assumes TaskNode.routing_node holds the manager-level node index for that visit.
  // If your TaskNode uses a different field, adjust here.
  for (const auto& tn : task_nodes) {
    // Guard: only positive nodes are customer visits (0 is depot in most setups).
    if (tn.routing_node > 0) {
      node_to_task[tn.routing_node] = &tn;
    }
  }
}

std::vector<std::vector<std::vector<std::string>>>
extract_routes_task_ids_from_firstschedule(
    const operations_research::Assignment* solution,
    const operations_research::RoutingModel& routing,
    const operations_research::RoutingIndexManager& manager,
    const std::vector<TaskNode>& task_nodes,
    int vehicles_per_day,
    int num_days)
{
  using operations_research::RoutingIndexManager;
  using operations_research::RoutingModel;

  std::unordered_map<int, const TaskNode*> node_to_task;
  std::unordered_map<std::string, int> day_to_idx;
  std::vector<std::string> idx_to_day;
  build_node_and_day_maps(task_nodes, node_to_task, day_to_idx, idx_to_day);

  std::vector<std::vector<std::vector<std::string>>> routes_by_day(
      num_days, std::vector<std::vector<std::string>>(vehicles_per_day));

  const int num_vehicles = vehicles_per_day * num_days;

  for (int v = 0; v < num_vehicles; ++v) {
    const int day_idx   = (vehicles_per_day > 0) ? (v / vehicles_per_day) : 0;
    const int route_idx = (vehicles_per_day > 0) ? (v % vehicles_per_day) : 0;

    const operations_research::RoutingModel::NodeIndex start = routing.Start(v);
    int64_t idx = solution->Value(routing.NextVar(start));
    while (!routing.IsEnd(idx)) {
      const int node = manager.IndexToNode(idx).value(); // manager-level node
      if (node != 0) {
        auto it = node_to_task.find(node);
        if (it != node_to_task.end()) {
          routes_by_day[day_idx][route_idx].push_back(it->second->task_id);
        }
      }
      idx = solution->Value(routing.NextVar(idx));
    }
  }
  return routes_by_day;
}

std::vector<std::vector<int>>
extract_route_minutes_from_firstschedule(
    const operations_research::Assignment* solution,
    const operations_research::RoutingModel& routing,
    const operations_research::RoutingIndexManager& /*manager*/,
    const operations_research::RoutingDimension& time_dim,
    int vehicles_per_day,
    int num_days)
{
  std::vector<std::vector<int>> mins_by_day(
      num_days, std::vector<int>(vehicles_per_day, 0));

  const int num_vehicles = vehicles_per_day * num_days;

  for (int v = 0; v < num_vehicles; ++v) {
    const int day_idx   = (vehicles_per_day > 0) ? (v / vehicles_per_day) : 0;
    const int route_idx = (vehicles_per_day > 0) ? (v % vehicles_per_day) : 0;

    const int64_t start = routing.Start(v);
    const int64_t end   = routing.End(v);

    int start_c = solution->Value(time_dim.CumulVar(start));
    int end_c   = solution->Value(time_dim.CumulVar(end));
    mins_by_day[day_idx][route_idx] = std::max(0, end_c - start_c);
  }
  return mins_by_day;
}

std::unordered_map<std::string, int>
build_taskid_date_index(
    const std::vector<TaskNode>& task_nodes,
    int /*num_days*/)
{
  // We return manager-level NodeIndex (int). Caller converts to RoutingIndex with manager.NodeToIndex(...)
  std::unordered_map<std::string, int> map_tid_day_to_node;
  map_tid_day_to_node.reserve(task_nodes.size() * 2);

  // Build day string -> day_idx
  std::vector<std::string> days;
  days.reserve(task_nodes.size() / 16 + 8);
  for (const auto& tn : task_nodes) days.push_back(tn.day);
  std::sort(days.begin(), days.end());
  days.erase(std::unique(days.begin(), days.end()), days.end());

  std::unordered_map<std::string, int> day_to_idx;
  for (int i = 0; i < (int)days.size(); ++i) day_to_idx[days[i]] = i;

  for (const auto& tn : task_nodes) {
    if (tn.routing_node <= 0) continue;
    auto it = day_to_idx.find(tn.day);
    if (it == day_to_idx.end()) continue;
    const int d = it->second;
    map_tid_day_to_node.emplace(key_tid_day(tn.task_id, d), tn.routing_node);
  }
  return map_tid_day_to_node;
}

std::unique_ptr<operations_research::Assignment>
read_assignment_from_routes_task_ids(
    const operations_research::RoutingModel& routing,
    const operations_research::RoutingIndexManager& manager,
    const std::vector<std::vector<std::vector<std::string>>>& routes_by_day,
    const std::unordered_map<std::string, int>& taskid_day_to_nodeindex,
    int vehicles_per_day,
    int num_days,
    bool ignore_inactive)
{
  using operations_research::Assignment;

  // Flatten to per-vehicle sequences in RoutingIndex space (no depot, only visits).
  const int num_vehicles = vehicles_per_day * num_days;
  std::vector<std::vector<int64_t>> per_vehicle(num_vehicles);

  auto get_nodeindex_for = [&](const std::string& tid, int day_idx) -> int {
    const std::string key = key_tid_day(tid, day_idx);
    auto it = taskid_day_to_nodeindex.find(key);
    if (it == taskid_day_to_nodeindex.end()) return -1;
    return it->second; // manager-level NodeIndex
  };

  for (int d = 0; d < num_days; ++d) {
    for (int r = 0; r < vehicles_per_day; ++r) {
      const int v = d * vehicles_per_day + r;
      const auto& seq = routes_by_day[d][r];
      auto& out = per_vehicle[v];
      out.reserve(seq.size());
      for (const auto& tid : seq) {
        int node_index = get_nodeindex_for(tid, d);
        if (node_index < 0) {
          // If missing, skip silently; caller should ensure completeness. You can add a CHECK/throw here if preferred.
          continue;
        }
        // Convert manager-level NodeIndex to RoutingIndex for this model:
        int64_t routing_index = manager.NodeToIndex(operations_research::RoutingIndexManager::NodeIndex(node_index)).value();
        out.push_back(routing_index);
      }
    }
  }

  // Build the warm-start assignment.
  std::unique_ptr<Assignment> assign(routing.solver()->MakeAssignment());
  std::unique_ptr<Assignment> tmp(routing.ReadAssignmentFromRoutes(per_vehicle, ignore_inactive));
  if (tmp) {
    assign.reset(tmp.release());
  } else {
    // If OR-Tools rejected the routes (infeasible under current constraints),
    // return an empty unique_ptr to signal caller to fall back to a cold solve.
    return {};
  }
  return assign;
}

// Converts a relaxed-solver JSON (one selected option per task) into a Seed
// with exactly one day_option per Task.
Seed seed_from_relaxed_json(const nlohmann::json& sol_json, const std::vector<Task>& original_tasks) {
  // Build task_id -> chosen date map from the solution JSON.
  // Assuming your parse_solution writes something like: assignments[day][route][...task_id...]
  // If you already output a flat list with {task_id,date}, use that directly.

  std::unordered_map<std::string,std::string> chosen_date;

  if (sol_json.contains("assigned") && sol_json["assigned"].is_array()) {
    for (const auto& a : sol_json["assigned"]) {
      std::string tid = a.value("task_id", "");
      std::string d   = a.value("date", "");
      if (!tid.empty() && !d.empty()) chosen_date[tid]=d;
    }
  } else if (sol_json.contains("routes_by_day")) {
    for (auto& day : sol_json["routes_by_day"].items()) {
      const std::string date = day.key();
      for (const auto& route : day.value()) {
        for (const auto& stop : route["stops"]) {
          std::string tid = stop.value("task_id","");
          if (!tid.empty()) chosen_date[tid]=date;
        }
      }
    }
  } // else adapt to your actual output structure

  Seed s; s.name = "relaxed_json";
  s.tasks.reserve(original_tasks.size());
  for (const auto& t : original_tasks) {
    const DayOption* picked=nullptr;
    auto it = chosen_date.find(t.task_id);
    if (it != chosen_date.end()) {
      for (const auto& o : t.day_options) if (o.date == it->second) { picked=&o; break; }
    }
    if (!picked) picked = t.day_options.empty() ? nullptr : &t.day_options.front();
    if (picked) s.tasks.push_back(choose_option(t, *picked));
  }
  return s;
}
