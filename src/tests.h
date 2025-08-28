#pragma once
#include "types.h"
#include "day_solver.h"
#include "cache.h"
#include "utils.h"
#include "sa.h"
#include <iostream>

// Quick smoke test: run the same day's VRP twice (cold vs warm).
inline void assert_warmup_working(const std::vector<TaskRef>& tasks,
                                  const TimeMatrix& TM,
                                  const SA_Config& cfg) {
    if (tasks.empty()) {
        std::cout << "[warm-start] no tasks, skipping.\n";
        return;
    }

    // Take tasks from the first week as our day sample
    int target_week = tasks.front().week;
    std::vector<TaskRef> day_tasks;
    for (auto& t : tasks) if (t.week == target_week) day_tasks.push_back(t);
    if (day_tasks.empty()) {
        std::cout << "[warm-start] no tasks in week " << target_week << "\n";
        return;
    }

    DaySolveParams cold;
    cold.time_limit_seconds = 3;
    cold.use_warm_start = false;

    DaySolveParams warm;
    warm.time_limit_seconds = 1;  // intentionally smaller
    warm.use_warm_start = true;

    DayConfig day_cfg = cfg.day_cfg;

    // --- Cold run ---
    long long t0 = NowMillis();
    DayResult r1 = solve_day(day_tasks, TM, day_cfg, cold, nullptr);
    long long t1 = NowMillis();

    // Prepare warm seed
    DayCacheValue v;
    v.result = r1;
    for (auto& route : r1.routes) {
        std::vector<int> seq;
        for (int id : route.sequence_client_indices) {
            if (id != 0) seq.push_back(id);
        }
        v.routes_global.push_back(std::move(seq));
    }

    DayWarmStart warm_seed;
    warm_seed.routes_global = v.routes_global;

    // --- Warm run ---
    long long t2 = NowMillis();
    DayResult r2 = solve_day(day_tasks, TM, day_cfg, warm, &warm_seed);
    long long t3 = NowMillis();

    std::cout << "[warm-start] tasks=" << day_tasks.size()
              << " cold=" << (t1 - t0) << "ms"
              << " warm=" << (t3 - t2) << "ms"
              << " cold_cost=" << r1.total_minutes
              << " warm_cost=" << r2.total_minutes
              << "\n";
}
