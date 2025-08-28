#pragma once
#include "types.h"
#include <unordered_map>
#include <map>

struct ScheduleState {
  // Decisions
  std::unordered_map<int,int>          canonical_weekday;      // schema -> 0..6
  std::unordered_map<std::string,int>  chosen_weekday;         // task_id -> 0..6

  // Derived
  std::map<int, std::vector<TaskRef>>  calendar;               // day_index -> tasks

  // Build {day_index -> tasks} from current chosen_weekday
  void build_calendar(const std::vector<TaskRef>& tasks, int total_weeks);

  // Initialize chosen_weekday (pick first allowed) and canonical_weekday (mode per schema)
  static void init_greedy(const std::vector<TaskRef>& tasks, ScheduleState& S);
};
