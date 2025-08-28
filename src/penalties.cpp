// penalties.cpp
#include "penalties.h"
#include <map>

int weekday_deviation_cost(const std::vector<TaskRef>& tasks,
                           const std::unordered_map<int,int>& k,
                           const std::unordered_map<std::string,int>& chosen,
                           const PenaltyWeights& W) {
  int c = 0;
  for (auto& t: tasks) {
    auto itc = chosen.find(t.task_id); if (itc==chosen.end()) continue;
    auto itk = k.find(t.schema_number); if (itk==k.end()) continue;
    c += W.w_dev * ring_distance7(itc->second, itk->second);
  }
  return c;
}

int variant_spacing_cost_same_week(const std::vector<TaskRef>& tasks,
                                   const std::unordered_map<std::string,int>& chosen,
                                   const PenaltyWeights& W) {
  // group by (schema_number, week) and only consider tasks with suffix != '\0'
  std::map<std::pair<int,int>, std::vector<int>> grp;
  for (auto& t: tasks) {
    if (t.suffix == '\0') continue;
    auto it = chosen.find(t.task_id); if (it==chosen.end()) continue;
    grp[{t.schema_number, t.week}].push_back(it->second);
  }
  int c = 0;
  for (auto& [key, wds] : grp) {
    for (size_t i=0;i<wds.size();++i) for (size_t j=i+1;j<wds.size();++j) {
      int gap = ring_distance7(wds[i], wds[j]);
      int smin = 2; // min spacing
      if (gap < smin) {
        int d = (smin - gap);
        c += W.w_space * d * d;
      }
    }
  }
  return c;
}

int vehicle_and_overtime_cost(int used_vehicles, int overtime_minutes, const PenaltyWeights& W) {
  return W.w_vu * used_vehicles + W.w_ot * overtime_minutes;
}
