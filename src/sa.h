// sa.h
#pragma once
#include "types.h"
#include "schedule_state.h"
#include "cache.h"
#include "penalties.h"
#include <random>

struct SA_Config {
  double T0=5.0, alpha=0.995;
  int iters=10000, reheats_every=3000;
  int total_weeks=12;
  int seed=42;
  bool verbose=false;
  int log_every=200;          // print every N iters when verbose
  DayConfig day_cfg;
  PenaltyWeights W;
};

struct GlobalCost {
  int routes_minutes = 0;
  int weekday_dev = 0;
  int spacing = 0;
  int ot_vu = 0;
  int total() const { return routes_minutes + weekday_dev + spacing + ot_vu; }
};


struct SA_Result {
  ScheduleState best_state;
  GlobalCost best_cost;
  // optional stats:
  int cache_hits=0, cache_misses=0;
};

SA_Result run_sa(const std::vector<TaskRef>& tasks,
                 const TimeMatrix& M,
                 const SA_Config& cfg);
