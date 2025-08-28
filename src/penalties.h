// penalties.h
#pragma once
#include <vector>
#include <unordered_map>
#include "types.h"

struct PenaltyWeights {
  int w_dev = 100;
  int w_space = 120;
  int w_vu = 1;
  int w_ot = 7;
};

inline int ring_distance7(int a, int b) {
  int d = std::abs(a-b);
  return std::min(d, 7-d);
}

int weekday_deviation_cost(const std::vector<TaskRef>& tasks,
                           const std::unordered_map<int,int>& canonical_weekday,
                           const std::unordered_map<std::string,int>& chosen_weekday,
                           const PenaltyWeights& W);

int variant_spacing_cost_same_week(const std::vector<TaskRef>& tasks,
                                   const std::unordered_map<std::string,int>& chosen_weekday,
                                   const PenaltyWeights& W);

int vehicle_and_overtime_cost(int used_vehicles, int overtime_minutes, const PenaltyWeights& W);
