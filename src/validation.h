// validation.h
#pragma once
#include <string>
#include <vector>
#include <stdexcept>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <regex>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <limits>
#include <iomanip>
#include <iostream>

// Include json so we can declare the overload that accepts raw JSON.
#include "third_party/json/single_include/nlohmann/json.hpp"

namespace vl {

// ---- Data model ----
struct Frequency {
  double value;             // can be fractional (e.g., 0.5 = biweekly)
  std::string unit;         // e.g., "week"
};

struct Address {
  std::string street;
  std::string city;
  std::string province;
};

struct Location {
  double lat;
  double lng;
};

struct DayOption {
  std::string date;   // "YYYY-MM-DD"
  int weekday;        // 0..6
};

struct Task {
    std::string task_id;
    int schema_number;
    Frequency frequency;
    Address address;
    Location location;
    double duration;
    int client_id;
    int week;
    int interval{1};
    bool is_intraweek{false};
    bool inferred_weekdays{false};
    std::vector<DayOption> day_options;   // [{date, weekday}, ...]
    std::vector<int>       weekday_options; // [weekday ints only]
};

// Square distance matrix in meters. Row/col 0 is the depot.
using Matrix = std::vector<std::vector<double>>;

struct Config {
  // From your JSON config keys:
  // {"MAX_HOURS_PER_DAY", 9}, {"VEHICLES_PER_DAY", 25}, {"TIME_LIMIT_SECONDS", 30}, {"USE_MATRIX_CACHE", true}
  double max_hours_per_route = 9.0; // hours per route/day
  int    max_routes_per_day  = 25;  // vehicles per day
  int    time_limit_seconds  = 30;  // solver time limit
  bool   use_matrix_cache    = false;
};

// Minimal view for horizon workload calc
struct TaskPlan {
  std::string task_id;
  int week;
  double duration;
  int client_id;
};

// Primary validator on parsed types
void validate_all(const std::vector<Task>& tasks,
                  const Matrix& matrix,
                  const Config& config);

// Convenience overload: accept raw JSON (nlohmann::json) and internally parse to types
void validate_all(const nlohmann::json& tasks_json,
                  const nlohmann::json& matrix_json,
                  const nlohmann::json& config_json);

// ---- Base field validation ----
void validate_tasks(const std::vector<Task>& tasks, std::size_t matrix_size);

// ---- Workload / matrix shape ----
void validate_total_workload(const std::vector<TaskPlan>& tasks,
                             const Matrix& matrix,
                             const Config& config);
void validate_unreachable_clients(const Matrix& matrix, double z_threshold = 3.0);

// ---- Additional validations ----
void validate_single_task_duration(const std::vector<Task>& tasks,
                                   const Config& config);
void validate_day_options_within_planning(const std::vector<Task>& tasks,
                                          int total_days);
void validate_weekly_capacity_projection(const std::vector<Task>& tasks,
                                         const Config& config);

// ---- Schema / recurrence ----
void validate_schema_week_gaps(const std::vector<Task>& tasks);
void validate_schema_duplicates(const std::vector<Task>& tasks);
void validate_dayoption_alignment(const std::vector<Task>& tasks);
void validate_assignment_possible(const std::vector<Task>& tasks, int planning_days);
void validate_dayoption_weekday_bounds(const std::vector<Task>& tasks);
void validate_week_number_range(const std::vector<Task>& tasks);
void validate_interval_positive(const std::vector<Task>& tasks);
void validate_schema_canonical_structure(const std::vector<Task>& tasks);
void validate_task_spacing_within_schema(const std::vector<Task>& tasks);

// ---- Matrix sanity ----
void validate_matrix_distance_range(const Matrix& matrix);
void validate_matrix_task_alignment(const std::vector<Task>& tasks, const Matrix& matrix);
void validate_no_placeholder_matrix(const Matrix& matrix);

// ---- Overload projection ----
void validate_dayoption_overload(const std::vector<Task>& tasks,
                                 const Config& config,
                                 int planning_days);

} // namespace vl
