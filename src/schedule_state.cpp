#include "schedule_state.h"
#include <array>
#include <algorithm>
#include <map>

void ScheduleState::build_calendar(const std::vector<TaskRef>& tasks, int /*total_weeks*/) {
  calendar.clear();
  for (const auto& t : tasks) {
    auto it = chosen_weekday.find(t.task_id);
    if (it == chosen_weekday.end()) continue;
    const int wd = it->second;                   // 0..6
    const int day_index = (t.week - 1) * 7 + wd; // 0-based
    calendar[day_index].push_back(t);
  }
}

void ScheduleState::init_greedy(const std::vector<TaskRef>& tasks, ScheduleState& S) {
  S.chosen_weekday.clear();
  S.canonical_weekday.clear();

  // Per-(week, weekday) load counters
  std::map<std::pair<int,int>, int> load; // ((week,wd) -> count)

  // 1) For each task, pick the allowed weekday with the smallest current load in its week.
  for (const auto& t : tasks) {
    int week = t.week;
    int best_wd = 0;
    int best_load = INT_MAX;
    if (!t.weekday_options.empty()) {
      for (int wd : t.weekday_options) {
        int l = load[{week, wd}];
        if (l < best_load) { best_load = l; best_wd = wd; }
      }
    } else {
      // Fallback if options empty
      best_wd = 0;
    }
    S.chosen_weekday[t.task_id] = best_wd;
    load[{week, best_wd}]++;
  }

  // 2) Canonical weekday per schema = mode of chosen weekdays for that schema
  std::unordered_map<int, std::array<int,7>> counts;
  for (const auto& t : tasks) {
    const int wd = S.chosen_weekday[t.task_id];
    counts[t.schema_number][wd] += 1;
  }
  for (auto& kv : counts) {
    int best_wd = 0, best_cnt = -1;
    for (int wd = 0; wd < 7; ++wd) {
      if (kv.second[wd] > best_cnt) { best_cnt = kv.second[wd]; best_wd = wd; }
    }
    S.canonical_weekday[kv.first] = best_wd;
  }
}
