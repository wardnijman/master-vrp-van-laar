#pragma once
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
    int num_vehicles);

// Build a 3D array [day][vehicleWithinDay][task_id] from a RoutingModel solution.
nlohmann::json build_routes_by_day_json(
    const operations_research::RoutingModel& routing,
    const operations_research::RoutingIndexManager& manager,
    const operations_research::Assignment* solution,
    int vehicles_per_day,
    int num_days,
    const std::vector<TaskNode>& task_nodes);



// ---------- filter to [start, end] window (inclusive) ----------
static json filter_tasks_to_window(const json& tasks_json, const std::string& start, const std::string& end) {
    json out = json::array();
    for (const auto& t : tasks_json) {
        if (!t.contains("day_options") || !t["day_options"].is_array()) continue;
        json kept = t;
        json opts = json::array();
        for (const auto& opt : t["day_options"]) {
            if (!opt.contains("date")) continue;
            std::string d = opt["date"].get<std::string>();
            if (ymd_in_range(d, start, end)) opts.push_back(opt);
        }
        if (!opts.empty()) {
            kept["day_options"] = std::move(opts);
            out.push_back(std::move(kept));
        }
    }
    return out;
}

// ---------- recurse solution JSON to collect {task_id, date} ----------
static void collect_assignments_recursive(const json& j,
                                          std::vector<std::pair<std::string,std::string>>& out) {
    if (j.is_object()) {
        bool has_id = j.contains("task_id") && j["task_id"].is_string();
        bool has_dt = j.contains("date")    && j["date"].is_string();
        if (has_id && has_dt) {
            out.emplace_back(j["task_id"].get<std::string>(), j["date"].get<std::string>());
        }
        for (auto it = j.begin(); it != j.end(); ++it) collect_assignments_recursive(it.value(), out);
    } else if (j.is_array()) {
        for (const auto& v : j) collect_assignments_recursive(v, out);
    }
}

// ---------- build canonical weekday map: schema -> weekday(1..7) ----------
static std::unordered_map<int,int> canonical_from_first_schedule(
        const json& first_solution,
        const json& full_tasks_json) {
    // Map task_id -> schema_number
    std::unordered_map<std::string,int> task_to_schema;
    task_to_schema.reserve(full_tasks_json.size()*2);
    for (const auto& t : full_tasks_json) {
        task_to_schema[t.at("task_id").get<std::string>()] = t.at("schema_number").get<int>();
    }

    std::vector<std::pair<std::string,std::string>> pairs;
    collect_assignments_recursive(first_solution, pairs);

    // count weekdays per schema
    std::unordered_map<int, std::array<int,8>> cnt; // 1..7
    for (auto& p : pairs) {
        auto it = task_to_schema.find(p.first);
        if (it == task_to_schema.end()) continue;
        int wd = weekday1_from_date(p.second);
        if (wd < 1 || wd > 7) continue;
        auto& arr = cnt[it->second];
        if (arr[0] == 0) arr.fill(0);
        arr[wd] += 1;
    }

    std::unordered_map<int,int> canon;
    canon.reserve(cnt.size());
    for (auto& kv : cnt) {
        int best_w = 1, best_c = -1;
        for (int w = 1; w <= 7; ++w) if (kv.second[w] > best_c) { best_c = kv.second[w]; best_w = w; }
        canon[kv.first] = best_w;
    }
    return canon;
}

// ---------- choose single day_option per task using canonical ----------
struct CanonPick {
    std::string date;
    int weekday1 = 0;  // 1..7
};
static CanonPick pick_day_for_task(const json& task,
                                   const std::unordered_map<int,int>& canon) {
    int schema = task.at("schema_number").get<int>();
    int target_wd = 0;
    if (auto it = canon.find(schema); it != canon.end()) target_wd = it->second;

    // Gather options {date, weekday}
    std::vector<std::pair<std::string,int>> opts;
    if (task.contains("day_options") && task["day_options"].is_array()) {
        for (const auto& o : task["day_options"]) {
            std::string d = o.value("date", "");
            int wd = o.contains("weekday") ? o["weekday"].get<int>() : weekday1_from_date(d);
            if (wd < 1 || wd > 7) wd = weekday1_from_date(d);
            if (!d.empty()) opts.emplace_back(d, wd);
        }
    }
    if (opts.empty()) throw std::runtime_error("Task " + task.at("task_id").get<std::string>() + " has no day_options.");

    auto score = [&](int wd) {
        if (target_wd == 0) return 0; // no canonical → neutral
        int dist = std::abs(wd - target_wd); dist = std::min(dist, 7 - dist);
        return dist;
    };

    // pick exact canonical if present; otherwise nearest by circular distance, then earliest date
    std::sort(opts.begin(), opts.end(), [&](auto& a, auto& b){
        int sa = score(a.second), sb = score(b.second);
        if (sa != sb) return sa < sb;
        return ymd_cmp(a.first, b.first) < 0;
    });

    return {opts.front().first, opts.front().second};
}

// ---------- convert full tasks.json + canon → vector<Seed::Task> (one day_option) ----------
static std::vector<Task> build_seed_from_canonical(const json& tasks_json,
                                                   const std::unordered_map<int,int>& canon) {
    std::vector<Task> out;
    out.reserve(tasks_json.size());
    for (const auto& t : tasks_json) {
        Task s{};
        s.task_id       = t.at("task_id").get<std::string>();
        s.schema_number = t.at("schema_number").get<int>();
        s.client_id     = t.at("client_id").get<int>();
        s.duration      = t.contains("duration_seconds")
                          ? std::llround(t.at("duration_seconds").get<double>() / 60.0)
                          : t.at("duration").get<int>();
        s.week          = t.at("week").get<int>();

        CanonPick pick = pick_day_for_task(t, canon);
        Task::DayOption d{};
        d.date    = pick.date;
        d.weekday = pick.weekday1;
        s.day_options = {d};

        out.push_back(std::move(s));
    }
    return out;
}

// ---------- light service-load balancing for weeks NOT in first window ----------
static void balance_seed_service_load(
    std::vector<Task>& seed_tasks,
    const json& orig_tasks_json,
    int vehicles_per_day,
    int route_limit_minutes)
{
    // Build lookup: task_id -> original day_options (date, weekday1)
    std::unordered_map<std::string, std::vector<std::pair<std::string,int>>> original_opts;
    original_opts.reserve(orig_tasks_json.size()*2);
    for (const auto& t : orig_tasks_json) {
        std::vector<std::pair<std::string,int>> v;
        if (t.contains("day_options") && t["day_options"].is_array()) {
            for (const auto& o : t["day_options"]) {
                std::string d = o.value("date", "");
                if (d.empty()) continue;
                int wd = o.contains("weekday") ? o["weekday"].get<int>() : weekday1_from_date(d);
                v.emplace_back(d, (wd<1||wd>7) ? weekday1_from_date(d) : wd);
            }
        }
        original_opts[t.at("task_id").get<std::string>()] = std::move(v);
    }

    // Day service cap heuristic (service only; travel not included)
    const double service_cap = vehicles_per_day * route_limit_minutes * 0.70; // 70% of total minutes as service budget

    // Group tasks by ISO week from date string (here we just use provided t.week)
    std::unordered_map<int, std::unordered_map<std::string,double>> week_day_load; // week -> date -> service
    std::unordered_map<std::string, std::vector<size_t>> date_index; // date -> indices of tasks in seed_tasks

    for (size_t i = 0; i < seed_tasks.size(); ++i) {
        const auto& t = seed_tasks[i];
        const auto& d = t.day_options.front().date;
        week_day_load[t.week][d] += t.duration;
        date_index[d].push_back(i);
    }

    // For each week, while any day is over service_cap + 45, move largest movable task to another allowed day in same week with min load
    const double tolerance = 45.0;
    for (auto& wk : week_day_load) {
        int week = wk.first;
        bool moved = true;
        int guard = 0;
        while (moved && guard++ < 2000) {
            moved = false;
            // find the most overloaded day in this week
            std::string worst_date; double worst_over = 0.0; double worst_load = 0.0;
            for (auto& kv : wk.second) {
                double load = kv.second;
                double over = load - service_cap;
                if (over > tolerance && load > worst_load) {
                    worst_load = load; worst_over = over; worst_date = kv.first;
                }
            }
            if (worst_date.empty()) break;

            // candidates on that date
            auto it_idx_vec = date_index.find(worst_date);
            if (it_idx_vec == date_index.end() || it_idx_vec->second.empty()) break;

            // pick largest movable task (has other options in same week)
            size_t best_i = SIZE_MAX; double best_size = -1.0;
            std::string best_new_date;
            for (size_t idx : it_idx_vec->second) {
                const Task& t = seed_tasks[idx];
                if (t.week != week) continue;
                auto& opts = original_opts[t.task_id];
                // find alternative dates in same week, not equal to current
                for (auto& od : opts) {
                    if (od.first == worst_date) continue;
                    // ensure same task's provided week matches; the json has week value already; assume options match same week
                    // choose the destination day with min current load
                    double dest_load = wk.second[od.first]; // default 0 if not exists
                    // record best destination for this task
                }
            }

            // now actually compute per-task best destination
            double best_gain = 0.0;
            for (size_t idx : it_idx_vec->second) {
                const Task& t = seed_tasks[idx];
                if (t.week != week) continue;
                auto& opts = original_opts[t.task_id];

                double size = t.duration;
                std::string best_dest;
                double dest_load_min = 1e18;

                for (auto& od : opts) {
                    if (od.first == worst_date) continue;
                    double dest_load = wk.second.count(od.first) ? wk.second[od.first] : 0.0;
                    if (dest_load < dest_load_min) { dest_load_min = dest_load; best_dest = od.first; }
                }
                if (best_dest.empty()) continue;

                double gain = std::min(size, worst_over); // moving this size reduces overload by up to size
                if (gain > best_gain) {
                    best_gain = gain; best_i = idx; best_new_date = best_dest; best_size = size;
                }
            }

            if (best_i != SIZE_MAX && !best_new_date.empty()) {
                // perform move
                Task& t = seed_tasks[best_i];
                std::string old_date = t.day_options.front().date;

                // update loads/index
                wk.second[old_date] -= best_size;
                wk.second[best_new_date] += best_size;

                auto& vec = date_index[old_date];
                vec.erase(std::remove(vec.begin(), vec.end(), best_i), vec.end());
                date_index[best_new_date].push_back(best_i);

                // write new assignment
                t.day_options.front().date    = best_new_date;
                t.day_options.front().weekday = weekday1_from_date(best_new_date);

                moved = true;
            }
        }
    }
}