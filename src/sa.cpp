#include "sa.h"
#include "day_solver.h"

#include <chrono>
#include <random>
#include <iostream>
#include <map>
#include <iomanip>
#include <climits>
#include <cmath>
#include "utils.h"

// Sum of service minutes for a set of tasks
static inline int sum_services(const std::vector<TaskRef> &v)
{
    int s = 0;
    for (const auto &t : v)
        s += t.service_minutes;
    return s;
}

// Convert DayResult.routes (global ids incl. depot at both ends) -> routes_global
// expected by warm start (per-vehicle client IDs, NO depot entries).
static std::vector<std::vector<int>> extract_routes_global(const DayResult &dr)
{
    std::vector<std::vector<int>> res;
    res.reserve(dr.routes.size());
    for (const auto &r : dr.routes)
    {
        const auto &seq = r.sequence_client_indices; // <-- use DayRoute field
        std::vector<int> g;
        g.reserve(seq.size());
        for (int node : seq)
        {
            if (node != 0) g.push_back(node); // drop depot
        }
        res.push_back(std::move(g));
    }
    return res;
}

// Lightweight counters for diagnostics
struct EvalStats
{
    int hits = 0;
    int misses = 0;
};

// --- cost eval over full calendar (uses day cache) ---
static GlobalCost eval_global(const ScheduleState &S,
                              const std::vector<TaskRef> &tasks,
                              const TimeMatrix &M,
                              DayCache &cache,
                              const SA_Config &cfg,
                              bool intensify,
                              EvalStats *stats)
{
    GlobalCost gc{};
    DaySolveParams sp{};
    sp.time_limit_seconds = intensify ? cfg.day_cfg.intensify_seconds
                                      : cfg.day_cfg.exploration_seconds;
    sp.log_search = cfg.verbose && intensify; // only spam OR-Tools logs when intensifying
    sp.use_warm_start = intensify;            // <— Warm start during intensification

    const int cap_minutes = cfg.day_cfg.vehicles * cfg.day_cfg.route_limit_minutes;

    for (const auto &kv : S.calendar)
    {
        const int day_idx = kv.first;
        const auto &day_tasks = kv.second;

        // Fast feasibility screen: if service alone exceeds per-day capacity, skip solve and penalize
        const int svc = sum_services(day_tasks);
        if (svc > cap_minutes)
        {
            const int overflow = svc - cap_minutes;
            if (cfg.verbose)
            {
                std::cout << "[overload] day=" << day_idx
                          << " tasks=" << day_tasks.size()
                          << " service=" << svc
                          << " cap=" << cap_minutes
                          << " overflow=" << overflow << "\n";
            }
            // Penalize: count all service minutes (as if routed) + overtime/vehicles proxy.
            gc.routes_minutes += svc;
            gc.ot_vu += (cfg.W.w_ot * overflow) + (cfg.W.w_vu * cfg.day_cfg.vehicles);
            continue;
        }

        DayCacheKey k{day_idx, DayCache::hash_task_ids(day_tasks)};
        DayCacheValue cv;
        const bool hit = cache.get(k, cv);

        if (stats) (hit ? stats->hits : stats->misses)++;

        // Prepare warm start (only if requested + hit)
        DayWarmStart warm{};
        const DayWarmStart *warm_ptr = nullptr;
        if (hit && sp.use_warm_start && !cv.routes_global.empty())
        {
            warm.routes_global = cv.routes_global;
            warm_ptr = &warm;
        }

        if (cfg.verbose)
        {
            std::cout << "[solve_day] day=" << day_idx
                      << " tasks=" << day_tasks.size()
                      << " tl=" << sp.time_limit_seconds << "s"
                      << " warm=" << (warm_ptr ? "yes" : "no")
                      << (hit ? " (cache hit)" : " (cache miss)")
                      << "\n";
        }

        DayResult dr = solve_day(day_tasks, M, cfg.day_cfg, sp, warm_ptr);

        if (!dr.feasible && cfg.verbose)
        {
            std::cout << "[solve_day] infeasible result for day=" << day_idx
                      << " tasks=" << day_tasks.size()
                      << " (soft horizon=" << cfg.day_cfg.route_limit_minutes << ")\n";
        }

        // Always refresh cache with newest result and the fresh warm seed
        DayCacheValue nv;
        nv.result = dr;
        nv.routes_global = extract_routes_global(dr); // derive warm-seed format from solution
        cache.put(k, nv);

        gc.routes_minutes += dr.total_minutes;
        gc.ot_vu += vehicle_and_overtime_cost(dr.used_vehicles, dr.overtime_minutes, cfg.W);
    }

    gc.weekday_dev = weekday_deviation_cost(tasks, S.canonical_weekday, S.chosen_weekday, cfg.W);
    gc.spacing = variant_spacing_cost_same_week(tasks, S.chosen_weekday, cfg.W);
    return gc;
}

// simple neighbor: move one task to its next allowed weekday (cyclic)
static inline void reassign_next_allowed(const TaskRef &t, ScheduleState &S)
{
    if (t.weekday_options.empty())
        return;
    const int cur = S.chosen_weekday[t.task_id];
    // find current position in options
    int pos = 0;
    while (pos < (int)t.weekday_options.size() && t.weekday_options[pos] != cur)
        ++pos;
    const int nxt = t.weekday_options[(pos + 1) % t.weekday_options.size()];
    S.chosen_weekday[t.task_id] = nxt;
}

// find first overloaded day (service-only test); return -1 if all OK
static inline int find_overloaded_day(const ScheduleState &S, const SA_Config &cfg)
{
    const int cap_minutes = cfg.day_cfg.vehicles * cfg.day_cfg.route_limit_minutes;
    for (const auto &kv : S.calendar)
    {
        int svc = 0;
        for (const auto &t : kv.second)
            svc += t.service_minutes;
        if (svc > cap_minutes)
            return kv.first;
    }
    return -1;
}

SA_Result run_sa(const std::vector<TaskRef> &tasks,
                 const TimeMatrix &M,
                 const SA_Config &cfg)
{

    std::mt19937_64 rng(cfg.seed);

    // --- initialize state greedily ---
    ScheduleState cur, best;
    ScheduleState::init_greedy(tasks, cur);
    cur.build_calendar(tasks, cfg.total_weeks);

    DayCache cache;
    EvalStats init_stats{};
    auto cur_cost = eval_global(cur, tasks, M, cache, cfg, /*intensify=*/false, &init_stats);
    auto best_cost = cur_cost;
    best = cur;

    if (cfg.verbose)
    {
        // quick day histogram
        std::map<int, int> hist;
        for (const auto &kv : cur.calendar)
            hist[(int)kv.second.size()]++;

        std::cout << "init: days=" << cur.calendar.size()
                  << " cache_hits=" << init_stats.hits
                  << " misses=" << init_stats.misses
                  << " cost=" << cur_cost.total()
                  << " (routes=" << cur_cost.routes_minutes
                  << " dev=" << cur_cost.weekday_dev
                  << " space=" << cur_cost.spacing
                  << " ot+vu=" << cur_cost.ot_vu << ")\n";
        std::cout << "day_size_histogram: ";
        for (const auto &kv : hist)
            std::cout << kv.first << "->" << kv.second << " ";
        std::cout << "\n";
    }

    double T = cfg.T0;
    std::uniform_int_distribution<size_t> pick(0, tasks.size() - 1);
    std::uniform_real_distribution<double> U(0.0, 1.0);

    // running cache stats
    int total_hits = init_stats.hits;
    int total_misses = init_stats.misses;

    for (int it = 1; it <= cfg.iters; ++it)
    {
        ScheduleState nxt = cur;

        // Prefer relieving an overloaded day if one exists
        int hot = find_overloaded_day(cur, cfg);
        if (hot >= 0)
        {
            // pick a random movable task from the hot day
            std::vector<const TaskRef *> candidates;
            auto itday = cur.calendar.find(hot);
            if (itday != cur.calendar.end())
            {
                for (const auto &tt : itday->second)
                    if (!tt.weekday_options.empty())
                        candidates.push_back(&tt);
            }
            if (!candidates.empty())
            {
                const TaskRef &t = *candidates[std::uniform_int_distribution<size_t>(0, candidates.size() - 1)(rng)];
                // move to next allowed weekday (cyclic), skipping current weekday
                int cur_wd = nxt.chosen_weekday[t.task_id];
                int pos = 0;
                while (pos < (int)t.weekday_options.size() && t.weekday_options[pos] != cur_wd)
                    ++pos;
                int nxt_wd = t.weekday_options[(pos + 1) % t.weekday_options.size()];
                nxt.chosen_weekday[t.task_id] = nxt_wd;
            }
            else
            {
                // fallback if no movable candidates
                for (int tries = 0; tries < 10; ++tries)
                {
                    const TaskRef &t = tasks[pick(rng)];
                    if (t.weekday_options.empty())
                        continue;
                    reassign_next_allowed(t, nxt);
                    break;
                }
            }
        }
        else
        {
            // No overloads: random single shift
            for (int tries = 0; tries < 10; ++tries)
            {
                const TaskRef &t = tasks[pick(rng)];
                if (t.weekday_options.empty())
                    continue;
                reassign_next_allowed(t, nxt);
                break;
            }
        }

        nxt.build_calendar(tasks, cfg.total_weeks);

        EvalStats step_stats{};
        auto nxt_cost = eval_global(nxt, tasks, M, cache, cfg, /*intensify=*/false, &step_stats);
        total_hits += step_stats.hits;
        total_misses += step_stats.misses;

        int dE = nxt_cost.total() - cur_cost.total();
        bool accept = (dE <= 0) || (U(rng) < std::exp(-dE / std::max(1e-6, T)));
        if (accept)
        {
            cur = std::move(nxt);
            cur_cost = nxt_cost;
            if (cur_cost.total() < best_cost.total())
            {
                best = cur;
                best_cost = cur_cost;
            }
        }

        if (cfg.verbose && (it % (cfg.log_every > 0 ? cfg.log_every : 200) == 0))
        {
            const double denom = (double)(total_hits + total_misses);
            const double hitrate = denom > 0.0 ? (double)total_hits / denom : 0.0;
            std::cout << "[it " << it << "] "
                      << "T=" << std::setprecision(3) << T
                      << " cur=" << cur_cost.total()
                      << " best=" << best_cost.total()
                      << " dE=" << dE
                      << " cache_hit_rate=" << std::fixed << std::setprecision(2) << (100.0 * hitrate) << "%\n";
        }

        T *= cfg.alpha;
        if (it % cfg.reheats_every == 0)
            T = cfg.T0;
    }

    // Final intensification pass that benefits from warm-starts:
    {
        EvalStats final_stats{};
        (void)final_stats;
        (void)total_hits; (void)total_misses; // keep counters if you want to print
        auto intensified = eval_global(best, tasks, M, cache, cfg, /*intensify=*/true, &final_stats);
        if (intensified.total() <= best_cost.total()) {
            best_cost = intensified;
        }
    }

    SA_Result res{best, best_cost};
    res.cache_hits = total_hits;
    res.cache_misses = total_misses;
    return res;
}
