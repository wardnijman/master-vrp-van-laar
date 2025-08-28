// day_solver.h
#pragma once
#include "types.h"

struct DaySolveParams {
  int time_limit_seconds = 2; // SA will vary this
  bool use_warm_start = true;
  bool log_search = false;
};

DayResult solve_day(const std::vector<TaskRef>& day_tasks,
                    const TimeMatrix& matrix,
                    const DayConfig& cfg,
                    const DaySolveParams& params,
                    const DayWarmStart* warm = nullptr);
