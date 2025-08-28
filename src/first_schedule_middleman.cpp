// first_schedule_middleman.cpp  — updated to use Hungarian-matched Jaccard diversity
#include "first_schedule_middleman.h"
#include "first_schedule_solver.h"

#include <future>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <numeric>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <functional>
#include <iostream>
#include <limits>
#include <cmath>
#include <optional>
#include <ortools/constraint_solver/routing.h>
#include <ortools/constraint_solver/routing_parameters.h>
#include "types.h"
#include "day_solver.h"

static bool g_mm_verbose = true; // or wire from mmcfg.verbose
static bool DEV = true;

#define MMLOG(...)                        \
    do                                    \
    {                                     \
        if (g_mm_verbose)                 \
        {                                 \
            fprintf(stderr, __VA_ARGS__); \
            fflush(stderr);               \
        }                                 \
    } while (0)

// --- small local helpers (no external deps needed) ---
namespace fsmm
{

    using namespace operations_research;

    using json = nlohmann::json;

    // ADD near the top of the helpers (inside namespace fsmm, same scope as other helpers)
    // Mutable overtime weight (adaptive); default used until we recompute after relaxed runs.
    static double g_W_OVERTIME = 2.5;
    static inline double clampd(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }

    constexpr double W_OVERTIME = 2.5; // weight for overtime minutes in insertion scoring
    constexpr double W_UTIL = 0.3;     // small bias toward lower post-insertion total minutes
    constexpr int BIG_COST = 1000000;  // sentinel for impossible placements

    struct InsEval
    {
        bool feasible = false;
        int pos = -1;
        int dtrav = 0;
        int new_total = 0;
        double comp = 0.0; // composite score we minimize
    };

    static const int BIG_COST_MIN = 30000; // used when we must discourage a move but keep running

    static inline int M_at(const std::vector<std::vector<int>> &M, int i, int j)
    {
        if (i < 0 || j < 0 || i >= (int)M.size() || j >= (int)M[i].size())
        {
            MMLOG("[panic] M index OOB: i=%d j=%d size=%zu row=%zu\n",
                  i, j, M.size(), (i >= 0 && i < (int)M.size()) ? M[i].size() : 0u);
            return BIG_COST_MIN; // don't crash; make it very expensive
        }
        return M[i][j];
    }

    static inline int client_of(const std::unordered_map<std::string, int> &task2client,
                                const std::string &tid)
    {
        auto it = task2client.find(tid);
        if (it == task2client.end())
        {
            MMLOG("[panic] missing task2client mapping for task_id=%s\n", tid.c_str());
            return -1; // will be guarded before indexing M
        }
        return it->second;
    }

    static inline bool check_client_bounds(const std::vector<std::vector<int>> &M, int cid)
    {
        if (cid < 0 || cid >= (int)M.size())
        {
            MMLOG("[panic] client index out of bounds: %d (M size=%zu)\n", cid, M.size());
            return false;
        }
        return true;
    }

    static bool sanity_check_M_vs_tasks(const std::unordered_map<std::string, int> &task2client,
                                        const std::vector<std::vector<int>> &M)
    {
        if (M.empty())
        {
            MMLOG("[panic] M is empty\n");
            return false;
        }
        int max_cid = 0;
        for (const auto &kv : task2client)
            max_cid = std::max(max_cid, kv.second);
        if ((int)M.size() <= max_cid)
        {
            MMLOG("[panic] M too small: rows=%zu, max_client_id=%d\n", M.size(), max_cid);
            return false;
        }
        for (size_t i = 0; i < M.size(); ++i)
        {
            if ((int)M[i].size() != (int)M.size())
            {
                MMLOG("[panic] M not square: row %zu has %zu cols, expected %zu\n",
                      i, M[i].size(), M.size());
                return false;
            }
        }
        return true;
    }

    // date helpers (yyyy-mm-dd)
    int date_diff_days(const std::string &start, const std::string &end)
    {
        std::tm a{}, b{};
        std::istringstream sa(start), sb(end);
        sa >> std::get_time(&a, "%Y-%m-%d");
        sb >> std::get_time(&b, "%Y-%m-%d");
        auto ta = std::mktime(&a), tb = std::mktime(&b);
        return int(std::difftime(tb, ta) / (60 * 60 * 24)) + 1;
    }
    std::string add_days(const std::string &start_date, int offset_days)
    {
        std::tm tm{};
        std::istringstream ss(start_date);
        ss >> std::get_time(&tm, "%Y-%m-%d");
        std::time_t t = std::mktime(&tm) + offset_days * 24 * 3600;
        std::tm *nt = std::localtime(&t);
        char buf[16];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d", nt);
        return std::string(buf);
    }

    static size_t count_tasks_in_rbd(const std::vector<std::vector<std::vector<std::string>>> &rbd)
    {
        std::unordered_set<std::string> s;
        for (const auto &day : rbd)
            for (const auto &r : day)
                for (const auto &t : r)
                    s.insert(t);
        return s.size();
    }
    static bool check_route_counts(const std::vector<std::vector<std::vector<std::string>>> &rbd, int vehicles_expected)
    {
        if (rbd.empty())
            return false;
        for (const auto &day : rbd)
            if ((int)day.size() != vehicles_expected)
                return false;
        return true;
    }
    // Total travel depot->...->depot, in minutes; guarded.
    static int route_travel_minutes(const std::vector<std::string> &route,
                                    const std::unordered_map<std::string, int> &task2client,
                                    const std::vector<std::vector<int>> &M)
    {
        int prev = 0; // depot
        int sum = 0;
        for (const auto &tid : route)
        {
            int cid = client_of(task2client, tid);
            if (DEV)
            {
                check_client_bounds(M, cid);
            }
            if (cid < 0)
                return BIG_COST_MIN; // bad task; punish path but don't crash
            sum += M_at(M, prev, cid);
            prev = cid;
        }
        sum += M_at(M, prev, 0);
        return sum;
    }

    static int route_total_minutes(const std::vector<std::string> &route,
                                   const std::unordered_map<std::string, int> &task2client,
                                   const std::unordered_map<std::string, int> &service_min,
                                   const std::vector<std::vector<int>> &M)
    {
        int s = 0;
        for (auto &tid : route)
            s += service_min.at(tid);
        return s + route_travel_minutes(route, task2client, M);
    }
    static std::pair<double, int> load_stats_minutes(const std::vector<std::vector<std::vector<std::string>>> &rbd,
                                                     const std::unordered_map<std::string, int> &task2client,
                                                     const std::unordered_map<std::string, int> &service_min,
                                                     const std::vector<std::vector<int>> &M)
    {
        long long sum = 0;
        int cnt = 0;
        int mx = 0;
        for (const auto &day : rbd)
            for (const auto &r : day)
            {
                int tot = route_total_minutes(r, task2client, service_min, M);
                sum += tot;
                ++cnt;
                mx = std::max(mx, tot);
            }
        double avg = (cnt ? (double)sum / (double)cnt : 0.0);
        return {avg, mx};
    }
    static int max_overage_minutes(const std::vector<std::vector<std::vector<std::string>>> &rbd,
                                   const std::unordered_map<std::string, int> &task2client,
                                   const std::unordered_map<std::string, int> &service_min,
                                   const std::vector<std::vector<int>> &M,
                                   int route_limit_minutes)
    {
        int mx = 0;
        for (const auto &day : rbd)
            for (const auto &r : day)
            {
                int tot = route_total_minutes(r, task2client, service_min, M);
                mx = std::max(mx, std::max(0, tot - route_limit_minutes));
            }
        return mx;
    }

    // Return delta travel minutes to insert t_cid at position pos in route.
    // Delta travel to insert client t_cid at position pos
    static int insertion_delta_travel(const std::vector<std::string> &route,
                                      int pos,
                                      int t_cid,
                                      const std::unordered_map<std::string, int> &task2client,
                                      const std::vector<std::vector<int>> &M)
    {
        if (!check_client_bounds(M, t_cid))
            return BIG_COST_MIN;

        auto cid_at = [&](int idx) -> int
        {
            if (idx < 0 || idx >= (int)route.size())
                return 0; // depot
            auto it = task2client.find(route[idx]);
            if (it == task2client.end())
            {
                MMLOG("[panic] delta: missing task2client for %s\n", route[idx].c_str());
                return 0; // treat as depot to keep running
            }
            int cid = it->second;
            if (!check_client_bounds(M, cid))
                return 0; // also treat as depot
            return cid;
        };

        int prev = (pos == 0) ? 0 : cid_at(pos - 1);
        int next = (pos == (int)route.size()) ? 0 : cid_at(pos);
        return M_at(M, prev, t_cid) + M_at(M, t_cid, next) - M_at(M, prev, next);
    }

    // REPLACE cheapest_insertion_eval(...) with this version (adds receiver fairness tie-break inputs via comp/new_total)
    // Composite “cheapest” insertion, respecting route caps and overtime.
    static InsEval cheapest_insertion_eval(const std::vector<std::string> &route,
                                           int t_cid, int t_srv,
                                           int route_limit_minutes, int ot_cap_minutes,
                                           const std::unordered_map<std::string, int> &task2client,
                                           const std::unordered_map<std::string, int> &service_min,
                                           const std::vector<std::vector<int>> &M)
    {
        // Guard: compute current totals safely
        int cur_service = 0;
        for (const auto &tid : route)
        {
            auto it = service_min.find(tid);
            if (it == service_min.end())
            {
                MMLOG("[panic] missing service_min for task_id=%s\n", tid.c_str());
                return {}; // infeasible
            }
            cur_service += it->second;
        }
        int cur_travel = route_travel_minutes(route, task2client, M);
        int cur_total = cur_service + cur_travel;

        InsEval best;
        best.comp = std::numeric_limits<double>::infinity();

        const int npos = (int)route.size() + 1;
        for (int p = 0; p < npos; ++p)
        {
            int dtrav = insertion_delta_travel(route, p, t_cid, task2client, M);
            int new_tot = cur_total + dtrav + t_srv;
            if (new_tot > route_limit_minutes + ot_cap_minutes)
                continue;

            int over = std::max(0, new_tot - route_limit_minutes);
            double comp = (double)dtrav + g_W_OVERTIME * (double)over + W_UTIL * (double)new_tot;

            if (comp < best.comp)
            {
                best.feasible = true;
                best.pos = p;
                best.dtrav = dtrav;
                best.new_total = new_tot;
                best.comp = comp;
            }
        }
        return best;
    }
    // Count how many receivers admit a feasible insertion of task (t_cid,t_srv).
    static int feasible_receiver_count(const std::vector<std::vector<std::string>> &receivers,
                                       int t_cid, int t_srv,
                                       int route_limit_minutes, int ot_cap_minutes,
                                       const std::unordered_map<std::string, int> &task2client,
                                       const std::unordered_map<std::string, int> &service_min,
                                       const std::vector<std::vector<int>> &M)
    {
        int cnt = 0;
        for (const auto &r : receivers)
        {
            InsEval ev = cheapest_insertion_eval(r, t_cid, t_srv,
                                                 route_limit_minutes, ot_cap_minutes,
                                                 task2client, service_min, M);
            if (ev.feasible)
                ++cnt;
        }
        return cnt;
    }

    // Extract [day][veh][task_id] from solver JSON. Throws if missing.
    std::vector<std::vector<std::vector<std::string>>> extract_routes_by_day(const json &sol)
    {
        if (!sol.contains("routes_by_day") || !sol["routes_by_day"].is_array())
            throw std::runtime_error("parse_solution JSON missing routes_by_day");
        std::vector<std::vector<std::vector<std::string>>> out;
        for (const auto &day : sol["routes_by_day"])
        {
            std::vector<std::vector<std::string>> day_routes;
            for (const auto &r : day)
            {
                std::vector<std::string> vec;
                if (r.contains("task_ids"))
                {
                    for (const auto &tid : r["task_ids"])
                        vec.push_back(tid.get<std::string>());
                }
                else if (r.is_array())
                {
                    for (const auto &tid : r)
                        vec.push_back(tid.get<std::string>());
                }
                day_routes.push_back(std::move(vec));
            }
            out.push_back(std::move(day_routes));
        }
        return out;
    }

    std::unordered_set<std::string> all_task_ids_from_tasks(const json &tasks)
    {
        std::unordered_set<std::string> s;
        for (const auto &t : tasks)
            s.insert(t["task_id"].get<std::string>());
        return s;
    }

    std::unordered_map<std::string, int> task_client_map(const json &tasks)
    {
        std::unordered_map<std::string, int> m;
        for (const auto &t : tasks)
            m.emplace(t["task_id"].get<std::string>(), t["client_id"].get<int>());
        return m;
    }
    std::unordered_map<std::string, int> task_service_minutes(const json &tasks)
    {
        std::unordered_map<std::string, int> m;
        for (const auto &t : tasks)
            m.emplace(t["task_id"].get<std::string>(), std::max(1, (int)t["duration"]));
        return m;
    }

    bool has_dropped_tasks(const std::vector<std::vector<std::vector<std::string>>> &routes_by_day,
                           const std::unordered_set<std::string> &all_tasks)
    {
        std::unordered_set<std::string> seen;
        for (const auto &day : routes_by_day)
            for (const auto &r : day)
                for (const auto &id : r)
                    seen.insert(id);
        return seen.size() != all_tasks.size();
    }

    struct BestIns
    {
        int rec_r = -1;
        int pos = -1;
        int delta_travel = 0;
        int new_total = 0;
        bool feasible = false;
    };

    static BestIns best_insertion_on_route(const std::vector<std::string> &route,
                                           int t_cid, int t_srv,
                                           int route_limit_minutes, int ot_cap_minutes,
                                           const std::unordered_map<std::string, int> &task2client,
                                           const std::unordered_map<std::string, int> &service_min,
                                           const std::vector<std::vector<int>> &M)
    {
        // current route totals
        int cur_service = 0;
        for (const auto &tid : route)
            cur_service += service_min.at(tid);
        int cur_travel = route_travel_minutes(route, task2client, M);
        int cur_total = cur_service + cur_travel;

        BestIns best;
        best.delta_travel = std::numeric_limits<int>::max();

        const int npos = (int)route.size() + 1;
        for (int p = 0; p < npos; ++p)
        {
            int dtrav = insertion_delta_travel(route, p, t_cid, task2client, M);
            int new_tot = cur_total + dtrav + t_srv;
            bool feas = (new_tot <= route_limit_minutes + ot_cap_minutes);
            // prefer feasible cheapest delta; if infeasible, still track but deprioritize
            if (!feas)
                continue;
            if (dtrav < best.delta_travel)
            {
                best.rec_r = 0; // filled by caller
                best.pos = p;
                best.delta_travel = dtrav;
                best.new_total = new_tot;
                best.feasible = true;
            }
        }
        return best;
    }

    // Direct per-day OR-Tools solve (25 vehicles) with hard per-route cap and small soft UB.
    // Returns true and fills out25 (routes of task_ids) when feasible.
    static bool direct_day_ortools_solve(const std::vector<std::vector<std::string>> &day26,
                                         int vehicles_per_day_target, // 25
                                         int route_limit_minutes,     // 9h * 60
                                         int ot_cap_minutes,          // e.g., 30–60
                                         const std::unordered_map<std::string, int> &task2client,
                                         const std::unordered_map<std::string, int> &service_min,
                                         const std::vector<std::vector<int>> &M,
                                         std::vector<std::vector<std::string>> &out25)
    {
        using namespace operations_research;
        // Gather unique tasks for the day
        std::vector<std::string> tasks;
        tasks.reserve(512);
        {
            std::unordered_set<std::string> seen;
            for (const auto &r : day26)
                for (const auto &t : r)
                    if (seen.insert(t).second)
                        tasks.push_back(t);
        }
        const int N = (int)tasks.size();
        if (N == 0)
        {
            out25.assign(vehicles_per_day_target, {});
            return true;
        }

        // Build mapping node->(task, client, service)
        struct Node
        {
            std::string tid;
            int cid;
            int srv;
        };
        std::vector<Node> nodes(N + 1); // 0 = depot
        nodes[0] = {"", 0, 0};
        for (int i = 0; i < N; ++i)
        {
            const auto &tid = tasks[i];
            nodes[i + 1] = {tid, task2client.at(tid), service_min.at(tid)};
        }

        // Manager/Model
        RoutingIndexManager mgr(N + 1, vehicles_per_day_target, RoutingIndexManager::NodeIndex(0));
        RoutingModel routing(mgr);

        // Transit: travel (minutes)
        auto dist_cb = [&mgr, &nodes, &M](int64_t from, int64_t to) -> int64_t
        {
            int fn = mgr.IndexToNode(from).value();
            int tn = mgr.IndexToNode(to).value();
            int fcid = nodes[fn].cid;
            int tcid = nodes[tn].cid;
            return M[fcid][tcid]; // minutes
        };
        int dist_idx = routing.RegisterTransitCallback(dist_cb);
        routing.SetArcCostEvaluatorOfAllVehicles(dist_idx);

        // Time = travel + service(from)
        auto time_cb = [&mgr, &nodes, &M](int64_t from, int64_t to) -> int64_t
        {
            int fn = mgr.IndexToNode(from).value();
            int tn = mgr.IndexToNode(to).value();
            int fcid = nodes[fn].cid;
            int tcid = nodes[tn].cid;
            int travel = M[fcid][tcid];
            return travel + nodes[fn].srv;
        };
        int time_idx = routing.RegisterTransitCallback(time_cb);

        const int cap = route_limit_minutes + ot_cap_minutes;
        routing.AddDimension(time_idx, /*slack=*/0, /*capacity=*/cap, /*fix_start=*/false, "Time");
        auto *time_dim = routing.GetMutableDimension("Time");
        for (int v = 0; v < vehicles_per_day_target; ++v)
        {
            int64_t s = routing.Start(v), e = routing.End(v);
            time_dim->CumulVar(s)->SetRange(0, route_limit_minutes);
            time_dim->CumulVar(e)->SetRange(0, cap);
            // gentle soft upper bound at hard day cap
            time_dim->SetCumulVarSoftUpperBound(e, route_limit_minutes, /*penalty=*/3);
        }

        // Force all tasks to be served (no disjunctions), but allow empty routes naturally.

        // Params
        RoutingSearchParameters sp = DefaultRoutingSearchParameters();
        sp.set_first_solution_strategy(FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION);
        sp.set_local_search_metaheuristic(LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
        sp.mutable_time_limit()->set_seconds(20);
        sp.set_log_search(false);

        const Assignment *sol = routing.SolveWithParameters(sp);
        if (!sol)
            return false;

        // Extract
        out25.assign(vehicles_per_day_target, {});
        for (int v = 0; v < vehicles_per_day_target; ++v)
        {
            int64_t idx = routing.Start(v);
            while (!routing.IsEnd(idx))
            {
                int64_t nxt = sol->Value(routing.NextVar(idx));
                int node = mgr.IndexToNode(idx).value();
                if (node != 0)
                    out25[v].push_back(nodes[node].tid);
                idx = nxt;
            }
        }
        return true;
    }

    // ADD this helper used by the drop-candidate scoring
    static double mergeability_proxy_for_route(const std::vector<std::string> &route_to_drop,
                                               const std::vector<std::vector<std::string>> &receivers,
                                               int route_limit_minutes, int ot_cap_minutes,
                                               const std::unordered_map<std::string, int> &task2client,
                                               const std::unordered_map<std::string, int> &service_min,
                                               const std::vector<std::vector<int>> &M)
    {
        if (route_to_drop.empty())
            return 0.0;
        double acc = 0.0;
        for (const auto &tid : route_to_drop)
        {
            int t_cid = task2client.at(tid);
            int t_srv = service_min.at(tid);
            double best = std::numeric_limits<double>::infinity();
            for (const auto &rec : receivers)
            {
                InsEval ev = cheapest_insertion_eval(rec, t_cid, t_srv,
                                                     route_limit_minutes, ot_cap_minutes,
                                                     task2client, service_min, M);
                if (ev.feasible)
                    best = std::min(best, (double)ev.dtrav);
            }
            if (!std::isfinite(best))
                best = (double)BIG_COST;
            acc += best;
        }
        return acc / (double)route_to_drop.size();
    }

    // Greedy regret-2 insertion with fairness tie-breaks (slack, then route size).
    // Insert all tasks from `pool` into `receivers`. Hard-first ordering + regret + 1-level ejection.
    // Insert all tasks from `pool` into `receivers` using hard-first ordering, regret,
    // and a single-level ejection when a task has no direct feasible receiver.
    //
    // "No feasible receiver" == for a given task, cheapest_insertion_eval(...) returns
    // ev.feasible==false for *every* receiver route (i.e., any insertion would exceed
    // route_limit_minutes + ot_cap_minutes or violate a hard constraint).
    static bool regret_insert_all(std::vector<std::vector<std::string>> &receivers,
                                  const std::vector<std::string> &pool,
                                  int route_limit_minutes, int ot_cap_minutes,
                                  const std::unordered_map<std::string, int> &task2client,
                                  const std::unordered_map<std::string, int> &service_min,
                                  const std::vector<std::vector<int>> &M)
    {
        struct Item
        {
            std::string tid;
            int cid;
            int srv;
            int deg;
        };
        std::vector<Item> items;
        items.reserve(pool.size());

        // Build items with safe lookups; deg = # of receivers that admit a feasible insertion
        for (const auto &tid : pool)
        {
            auto itC = task2client.find(tid);
            auto itS = service_min.find(tid);
            if (itC == task2client.end() || itS == service_min.end())
            {
                MMLOG("[panic] pool task missing mapping: tid=%s (cid? %d present, srv? %d present)\n",
                      tid.c_str(), itC != task2client.end(), itS != service_min.end());
                return false; // cannot proceed robustly
            }
            int cid = itC->second;
            int srv = itS->second;
            int deg = feasible_receiver_count(receivers, cid, srv,
                                              route_limit_minutes, ot_cap_minutes,
                                              task2client, service_min, M);
            items.push_back({tid, cid, srv, deg});
        }

        // Hard-first: fewest feasible receivers first (deg↑ harder), then larger service first
        std::sort(items.begin(), items.end(), [](const Item &a, const Item &b)
                  {
        if (a.deg != b.deg) return a.deg < b.deg;
        return a.srv > b.srv; });

        auto route_total = [&](const std::vector<std::string> &r) -> int
        {
            int s = 0;
            for (const auto &t : r)
            {
                auto it = service_min.find(t);
                if (it == service_min.end())
                {
                    MMLOG("[panic] missing service_min for %s\n", t.c_str());
                    return BIG_COST_MIN;
                }
                s += it->second;
            }
            int tmin = route_travel_minutes(r, task2client, M);
            return (tmin >= BIG_COST_MIN) ? BIG_COST_MIN : (s + tmin);
        };

        for (size_t it = 0; it < items.size(); ++it)
        {
            const auto &x = items[it];

            // Gather best and second-best feasible insertions across all receivers (for regret)
            struct Choice
            {
                int ridx = -1;
                int pos = 0;
                int dtrav = 0;
                int newtot = 0;
                double comp = 0.0;
                bool ok = false;
            };
            Choice best, second;

            for (int r = 0; r < (int)receivers.size(); ++r)
            {
                InsEval ev = cheapest_insertion_eval(receivers[r], x.cid, x.srv,
                                                     route_limit_minutes, ot_cap_minutes,
                                                     task2client, service_min, M);
                if (!ev.feasible)
                    continue;

                Choice c;
                c.ridx = r;
                c.pos = ev.pos;
                c.dtrav = ev.dtrav;
                c.newtot = ev.new_total;
                c.comp = ev.comp;
                c.ok = true;
                if (!best.ok || c.comp < best.comp)
                {
                    second = best;
                    best = c;
                }
                else if (!second.ok || c.comp < second.comp)
                {
                    second = c;
                }
            }

            // If no direct feasible receiver exists, attempt a 1-level ejection chain:
            //  - choose a target receiver that would be "least bad" if we forced x at the tail,
            //  - eject the smallest-service "victim" from it,
            //  - relocate that victim to some other receiver feasibly,
            //  - then re-try inserting x into the target.
            if (!best.ok)
            {
                bool placed = false;

                // Heuristic target receiver: minimize (tail Δtravel + weighted overtime overflow)
                int tgt = -1;
                double tgt_score = std::numeric_limits<double>::infinity();
                for (int r = 0; r < (int)receivers.size(); ++r)
                {
                    int cur = route_total(receivers[r]);
                    if (cur >= BIG_COST_MIN)
                        continue;
                    int dtrav_tail = insertion_delta_travel(receivers[r], (int)receivers[r].size(), x.cid, task2client, M);
                    int new_tot = cur + dtrav_tail + x.srv;
                    int over = std::max(0, new_tot - route_limit_minutes);
                    double score = (double)dtrav_tail + g_W_OVERTIME * over;
                    if (score < tgt_score)
                    {
                        tgt_score = score;
                        tgt = r;
                    }
                }

                if (tgt >= 0 && !receivers[tgt].empty())
                {
                    // Pick a cheap-to-move victim from tgt (min service)
                    int victim_idx = -1, victim_srv = INT_MAX;
                    for (int j = 0; j < (int)receivers[tgt].size(); ++j)
                    {
                        auto itSrv = service_min.find(receivers[tgt][j]);
                        if (itSrv == service_min.end())
                            continue;
                        if (itSrv->second < victim_srv)
                        {
                            victim_srv = itSrv->second;
                            victim_idx = j;
                        }
                    }

                    if (victim_idx >= 0)
                    {
                        std::string victim = receivers[tgt][victim_idx];

                        // Remove victim from tgt (make a working copy so we can rollback cleanly)
                        std::vector<std::string> tgt_copy = receivers[tgt];
                        tgt_copy.erase(tgt_copy.begin() + victim_idx);

                        int v_cid = client_of(task2client, victim);
                        int v_srv = service_min.at(victim);
                        if (v_cid >= 0)
                        {
                            int best_r2 = -1;
                            InsEval best_ev2;
                            for (int r2 = 0; r2 < (int)receivers.size(); ++r2)
                                if (r2 != tgt)
                                {
                                    InsEval ev2 = cheapest_insertion_eval(receivers[r2], v_cid, v_srv,
                                                                          route_limit_minutes, ot_cap_minutes,
                                                                          task2client, service_min, M);
                                    if (!ev2.feasible)
                                        continue;
                                    if (best_r2 < 0 || ev2.comp < best_ev2.comp)
                                    {
                                        best_r2 = r2;
                                        best_ev2 = ev2;
                                    }
                                }

                            if (best_r2 >= 0)
                            {
                                // Commit victim relocation
                                receivers[tgt] = std::move(tgt_copy);
                                receivers[best_r2].insert(receivers[best_r2].begin() + best_ev2.pos, victim);

                                // Re-evaluate inserting x into tgt
                                InsEval evX = cheapest_insertion_eval(receivers[tgt], x.cid, x.srv,
                                                                      route_limit_minutes, ot_cap_minutes,
                                                                      task2client, service_min, M);
                                if (evX.feasible)
                                {
                                    receivers[tgt].insert(receivers[tgt].begin() + evX.pos, x.tid);
                                    placed = true;
                                }
                                else
                                {
                                    // Rollback victim if x still doesn't fit
                                    auto &r2 = receivers[best_r2];
                                    auto itv = std::find(r2.begin(), r2.end(), victim);
                                    if (itv != r2.end())
                                        r2.erase(itv);
                                    auto &rt = receivers[tgt];
                                    int ins_pos = std::min<int>(std::max(0, victim_idx), (int)rt.size());
                                    rt.insert(rt.begin() + ins_pos, victim);
                                }
                            }
                        }
                    }
                }

                if (!placed)
                {
                    MMLOG("[regret] task %s has no feasible receiver even after ejection\n", x.tid.c_str());
                    return false; // still blocked
                }
                continue; // next task
            }

            // Regret choice: if second exists, bias by regret; if regret tiny, prefer the route with more slack
            // Regret choice: if second exists, bias by regret; if regret tiny, prefer route with more slack
            // Choose best vs. second
            Choice chosen = best;
            if (second.ok)
            {
                double regret = second.comp - best.comp;
                if (regret < 5.0 && (second.newtot + 10) < best.newtot)
                    chosen = second;
            }

            // Guard the route index just in case
            if (chosen.ridx < 0 || chosen.ridx >= (int)receivers.size())
            {
                MMLOG("[panic] chosen.ridx out of range: %d (receivers=%zu)\n",
                      chosen.ridx, receivers.size());
                return false;
            }

            // Clamp insertion position to current route length
            auto &route = receivers[chosen.ridx]; // or receivers.at(...) while debugging
            int ins_pos = chosen.pos;
            if (ins_pos < 0)
                ins_pos = 0;
            if (ins_pos > (int)route.size())
                ins_pos = (int)route.size();

            route.insert(route.begin() + ins_pos, x.tid);
        }

        return true;
    }

    // Single-pass 1-opt on each receiver route (only travel deltas)
    static void polish_routes_local(std::vector<std::vector<std::string>> &routes,
                                    const std::unordered_map<std::string, int> &task2client,
                                    const std::vector<std::vector<int>> &M)
    {
        for (auto &r : routes)
        {
            if (r.size() < 3)
                continue;

            bool improved = true;
            int guard = 0;
            while (improved && guard++ < 2) // a couple of passes is plenty
            {
                improved = false;
                for (size_t i = 0; i + 1 < r.size(); ++i)
                {
                    auto safe_cid = [&](int idx) -> int
                    {
                        if (idx < 0 || idx >= (int)r.size())
                            return 0; // depot
                        auto it = task2client.find(r[idx]);
                        if (it == task2client.end())
                        {
                            MMLOG("[panic] polish: missing task2client for %s\n", r[idx].c_str());
                            return -1;
                        }
                        int cid = it->second;
                        if (!check_client_bounds(M, cid))
                            return -1;
                        return cid;
                    };

                    int a = (i == 0) ? 0 : safe_cid((int)i - 1);
                    int b = safe_cid((int)i);
                    int c = safe_cid((int)i + 1);
                    int d = ((i + 2) < r.size()) ? safe_cid((int)i + 2) : 0;

                    if (a < 0 || b < 0 || c < 0 || d < 0)
                    {
                        // skip this swap opportunity if any id is unsafe
                        continue;
                    }

                    int cur = M_at(M, a, b) + M_at(M, b, c) + M_at(M, c, d);
                    int swp = M_at(M, a, c) + M_at(M, c, b) + M_at(M, b, d);

                    if (swp < cur)
                    {
                        std::swap(r[i], r[i + 1]);
                        improved = true;
                    }
                }
            }
        }
    }

    static bool try_drop_one_day_regret(const std::vector<std::vector<std::string>> &day26,
                                        int route_limit_minutes, int ot_cap_minutes,
                                        const std::unordered_map<std::string, int> &task2client,
                                        const std::unordered_map<std::string, int> &service_min,
                                        const std::vector<std::vector<int>> &M,
                                        std::vector<std::vector<std::string>> &out25,
                                        int K_candidates = 5)
    {
        const int V26 = (int)day26.size();
        const int V25 = V26 - 1;

        // Precompute service & travel
        std::vector<int> route_service(V26, 0), route_travel(V26, 0), route_total(V26, 0);
        for (int r = 0; r < V26; ++r)
        {
            int s = 0;
            for (const auto &tid : day26[r])
                s += service_min.at(tid);
            int t = route_travel_minutes(day26[r], task2client, M);
            route_service[r] = s;
            route_travel[r] = t;
            route_total[r] = s + t;
        }

        struct Cand
        {
            int idx;
            double score;
            long long slack_sum;
        };
        std::vector<Cand> cands;
        cands.reserve(V26);

        for (int q = 0; q < V26; ++q)
        {
            // Receivers = all except q
            std::vector<int> rec_idx;
            rec_idx.reserve(V25);
            for (int r = 0; r < V26; ++r)
                if (r != q)
                    rec_idx.push_back(r);

            // Slack filter
            long long S_slack = 0;
            for (int r : rec_idx)
            {
                int slack = route_limit_minutes - route_total[r];
                S_slack += std::max(0, slack + ot_cap_minutes);
            }
            if (route_service[q] > S_slack)
            {
                MMLOG("[cand] drop=%d REJECT slack routeS=%d S_slack=%lld\n", q, route_service[q], S_slack);
                continue;
            }

            // Mergeability proxy
            std::vector<std::vector<std::string>> rec_routes;
            rec_routes.reserve(V25);
            for (int r : rec_idx)
                rec_routes.push_back(day26[r]);

            double Mproxy = mergeability_proxy_for_route(day26[q], rec_routes,
                                                         route_limit_minutes, ot_cap_minutes,
                                                         task2client, service_min, M);

            double score = 1.0 * route_service[q] + 0.6 * route_travel[q] + 0.4 * Mproxy;
            cands.push_back({q, score, S_slack});
        }

        if (cands.empty())
            return false;

        std::sort(cands.begin(), cands.end(), [](const Cand &a, const Cand &b)
                  {
                      if (a.score != b.score)
                          return a.score < b.score;
                      return a.slack_sum > b.slack_sum; // tie-break: more global slack
                  });

        int tried = 0;
        for (const auto &cand : cands)
        {
            if (tried++ >= K_candidates)
                break;

            // Build receivers for this candidate
            std::vector<std::vector<std::string>> receivers;
            receivers.reserve(V25);
            for (int r = 0; r < V26; ++r)
                if (r != cand.idx)
                    receivers.push_back(day26[r]);
            const auto &pool = day26[cand.idx];

            MMLOG("[try] drop=%d score=%.2f slack=%lld | pool=%zu\n",
                  cand.idx, cand.score, cand.slack_sum, pool.size());

            if (regret_insert_all(receivers, pool, route_limit_minutes, ot_cap_minutes,
                                  task2client, service_min, M))
            {
                // Optional polish: cheap per-route 1-opt travel reduction (pure delta edges)
                // polish_routes_local(receivers, task2client, M);

                out25 = std::move(receivers);
                return true;
            }
            MMLOG("[try] drop=%d failed\n", cand.idx);
        }
        return false;
    }

    static bool drop_with_ortools_polish(const std::vector<std::vector<std::string>> &day26,
                                         int route_limit_minutes, int ot_cap_minutes,
                                         const std::unordered_map<std::string, int> &task2client,
                                         const std::unordered_map<std::string, int> &service_min,
                                         const std::vector<std::vector<int>> &M,
                                         std::vector<std::vector<std::string>> &out25)
    {
        // 1) Try to build a greedy 25 solution once (K=7, +OT). If it fails, we cannot even seed.
        std::vector<std::vector<std::string>> base25;
        if (!try_drop_one_day_regret(day26, route_limit_minutes, ot_cap_minutes + 10,
                                     task2client, service_min, M, base25, 7))
        {
            MMLOG("[polish] cannot form base25 seed\n");
            return false;
        }

        // 2) Hand off to your day VRP solver with 25 vehicles, using base25 as warm routes
        DayConfig dc{};
        dc.vehicles = (int)base25.size();
        dc.route_limit_minutes = route_limit_minutes;
        dc.soft_upper_cost = 1;
        dc.span_coeff = 0;

        DaySolveParams sp{};
        sp.time_limit_seconds = 12; // short polish
        sp.log_search = false;
        sp.use_warm_start = true;

        // Convert base25 → DayWarmStart.routes_global (client indices, no depot)
        DayWarmStart warm{};
        warm.routes_global.reserve(base25.size());
        for (const auto &r : base25)
        {
            std::vector<int> g;
            g.reserve(r.size());
            for (const auto &tid : r)
                g.push_back(task2client.at(tid));
            warm.routes_global.push_back(std::move(g));
        }

        // Build TaskRef list for the day
        std::vector<TaskRef> refs;
        {
            std::unordered_set<std::string> seen;
            for (const auto &r : day26)
                for (const auto &tid : r)
                    seen.insert(tid);
            refs.reserve(seen.size());
            for (const auto &tid : seen)
            {
                TaskRef tr{};
                tr.task_id = tid;
                tr.client_idx = task2client.at(tid);
                tr.service_minutes = service_min.at(tid);
                refs.push_back(tr);
            }
        }

        TimeMatrix TM;
        TM.n = (int)M.size();
        TM.m.reserve(TM.n * TM.n);
        for (int i = 0; i < TM.n; ++i)
            for (int j = 0; j < TM.n; ++j)
                TM.m.push_back(M[i][j]);

        DayResult dr = solve_day(refs, TM, dc, sp, &warm);
        if (!dr.feasible)
        {
            MMLOG("[polish] OR-Tools polish infeasible\n");
            return false;
        }

        // Map back to task_ids per route
        out25.clear();
        out25.resize(dr.routes.size());
        // You already store DayRoute.sequence_client_indices (global ids including depot).
        for (size_t r = 0; r < dr.routes.size(); ++r)
        {
            const auto &seq = dr.routes[r].sequence_client_indices;
            for (int node : seq)
            {
                if (node == 0)
                    continue;
                // Need reverse map client_idx -> any task_id in the day with that client
                // If multiple tasks share client, we match greedily by remaining counts
                // Simple: build multimap client_idx -> list<tid> from day26 beforehand.
            }
        }
        // ^ If client duplicates are common, keep a per-client queue of remaining task_ids from base25 and fill.

        return true;
    }

    // New regret-based elimination: returns success flag and fills res.
    // Adaptive OT, detailed logs, and robust fallback ladder.
    static bool eliminate_one_route_per_day_regret(
        const std::vector<std::vector<std::vector<std::string>>> &routes_by_day26,
        const std::unordered_map<std::string, int> &service_min,
        const std::unordered_map<std::string, int> &task2client,
        const std::vector<std::vector<int>> &M,
        int route_limit_minutes,
        int ot_cap_minutes, // e.g., 45
        std::vector<std::vector<std::vector<std::string>>> &res_out)
    {
        const int days = (int)routes_by_day26.size();
        if (days == 0)
        {
            res_out.clear();
            return true;
        }

        const int veh26 = (int)routes_by_day26[0].size();
        const int veh25 = veh26 - 1;

        std::vector<std::vector<std::vector<std::string>>> res(
            days, std::vector<std::vector<std::string>>(veh25));

        for (int d = 0; d < days; ++d)
        {
            const auto &day26 = routes_by_day26[d];

            // Quick day diagnostics
            auto r_service = [&](int r)
            { int s=0; for (const auto& t : day26[r]) s += service_min.at(t); return s; };
            auto r_travel = [&](int r)
            { return route_travel_minutes(day26[r], task2client, M); };

            long long totS = 0, totT = 0;
            for (int r = 0; r < veh26; ++r)
            {
                totS += r_service(r);
                totT += r_travel(r);
            }

            // Adaptive OT bump: compute aggregate deficit for going from 26→25 routes
            const long long base_cap_25 = 1LL * route_limit_minutes * veh25;
            const long long base_cap_25_with_ot = base_cap_25 + 1LL * veh25 * ot_cap_minutes;
            const long long deficit = (totS + totT) - base_cap_25_with_ot; // >0 means shortage
            int bump = 0;
            if (deficit > 0)
            {
                // Distribute deficit across the 25 vehicles (ceil), cap to something sensible.
                bump = (int)((deficit + veh25 - 1) / veh25);
                bump = std::min(bump, 60); // don't explode; tune if needed
            }

            MMLOG("[elim] day=%d V26=%d S=%lld T=%lld cap25=%lld ot_cap=%d (deficit=%lld bump=%d)\n",
                  d, veh26, totS, totT, base_cap_25, ot_cap_minutes, std::max(0LL, deficit), bump);

            std::vector<std::vector<std::string>> day25;

            // Ladder 1: K=5 with original OT cap
            bool ok = try_drop_one_day_regret(day26, route_limit_minutes, ot_cap_minutes,
                                              task2client, service_min, M, day25, /*K_candidates=*/5);

            // Ladder 2: K=7 with original OT cap
            if (!ok)
            {
                MMLOG("[elim] day=%d retry K=7\n", d);
                ok = try_drop_one_day_regret(day26, route_limit_minutes, ot_cap_minutes,
                                             task2client, service_min, M, day25, /*K_candidates=*/7);
            }

            // Ladder 3: K=7 with boosted OT (max of +10 and adaptive bump)
            if (!ok)
            {
                const int extra = std::max(10, bump);
                MMLOG("[elim] day=%d retry K=7 +%d OT (from %d → %d)\n",
                      d, extra, ot_cap_minutes, ot_cap_minutes + extra);
                ok = try_drop_one_day_regret(day26, route_limit_minutes, ot_cap_minutes + extra,
                                             task2client, service_min, M, day25, /*K_candidates=*/7);
            }

            // Ladder 4: direct per-day OR-Tools solve with 25 vehicles (no warm start)
            if (!ok)
            {
                const int dir_ot = std::max(ot_cap_minutes, std::max(30, bump)); // be a bit generous here
                MMLOG("[elim] day=%d fallback OR-Tools DIRECT 25-vehicle solve (OT=%d)\n", d, dir_ot);
                ok = direct_day_ortools_solve(day26,
                                              /*vehicles_per_day_target=*/veh25,
                                              route_limit_minutes,
                                              /*ot_cap_minutes=*/dir_ot,
                                              task2client, service_min, M, day25);
            }

            // Ladder 5: warm-start polish (only if we at least have a greedy base25)
            // Note: drop_with_ortools_polish expects we provide a candidate 'day25' to warm-start.
            if (!ok)
            {
                MMLOG("[elim] day=%d fallback OR-Tools cleanup (warm-start polish)\n", d);
                ok = drop_with_ortools_polish(day26, route_limit_minutes, ot_cap_minutes,
                                              task2client, service_min, M, day25);
            }

            if (!ok)
            {
                MMLOG("[elim] day=%d FAILED\n", d);
                return false;
            }

            MMLOG("[elim] day=%d OK routes=%d\n", d, (int)day25.size());
            res[d] = std::move(day25);
        }

        res_out = std::move(res);
        return true;
    }

    struct Metrics
    {
        double travel = 0, overtime = 0;
    };

    Metrics compute_metrics(const std::vector<std::vector<std::vector<std::string>>> &routes_by_day,
                            const std::unordered_map<std::string, int> &task2client,
                            const std::unordered_map<std::string, int> &service_min,
                            const std::vector<std::vector<int>> &matrix_minutes,
                            int route_limit_minutes)
    {
        Metrics m;
        for (const auto &day : routes_by_day)
        {
            for (const auto &r : day)
            {
                int travel = route_travel_minutes(r, task2client, matrix_minutes);
                int service = 0;
                for (const auto &tid : r)
                    service += service_min.at(tid);
                int total = travel + service;
                m.travel += travel;
                if (total > route_limit_minutes)
                    m.overtime += (total - route_limit_minutes);
            }
        }
        return m;
    }

    std::string assignment_signature(const std::vector<std::vector<std::vector<std::string>>> &routes_by_day)
    {
        // task_id -> (day, route_idx)
        std::vector<std::string> parts;
        for (int d = 0; d < (int)routes_by_day.size(); ++d)
        {
            for (int r = 0; r < (int)routes_by_day[d].size(); ++r)
            {
                for (const auto &tid : routes_by_day[d][r])
                {
                    std::ostringstream os;
                    os << tid << "|" << d << "|" << r;
                    parts.push_back(os.str());
                }
            }
        }
        std::sort(parts.begin(), parts.end());
        std::ostringstream all;
        for (auto &p : parts)
        {
            all << p << ";";
        }
        // hash to compact
        return std::to_string(std::hash<std::string>{}(all.str()));
    }

    // Build fixed-day tasks: for each task, keep only the day_option that matches day index in assignment
    // Requires a min_date to translate day index -> date string present in original day_options.
    json build_fixed_day_tasks(const json &tasks,
                               const std::vector<std::vector<std::vector<std::string>>> &routes_by_day,
                               const std::string &min_date)
    {
        // Map task -> chosen day index
        std::unordered_map<std::string, int> chosen_day;
        for (int d = 0; d < (int)routes_by_day.size(); ++d)
            for (const auto &r : routes_by_day[d])
                for (const auto &tid : r)
                    chosen_day[tid] = d;

        json out = json::array();
        for (const auto &t : tasks)
        {
            const std::string tid = t["task_id"].get<std::string>();
            int d = chosen_day.at(tid);
            std::string date = add_days(min_date, d);
            json tcopy = t;
            // filter day_options to only the chosen date
            json kept = json::array();
            for (const auto &opt : t["day_options"])
            {
                if (opt["date"].get<std::string>() == date)
                    kept.push_back(opt);
            }
            tcopy["day_options"] = kept;
            out.push_back(std::move(tcopy));
        }
        return out;
    }

    // Find global min_date from original tasks (needed to compute chosen dates)
    std::string find_min_date(const json &tasks)
    {
        std::string min_date = "9999-12-31";
        for (const auto &t : tasks)
            for (const auto &opt : t["day_options"])
            {
                const std::string d = opt["date"].get<std::string>();
                if (d < min_date)
                    min_date = d;
            }
        return min_date;
    }

    // Drop one route per day (26->25) by removing the lightest route and appending its tasks
    // to the currently lightest remaining route (by service only, quick heuristic).
    std::vector<std::vector<std::vector<std::string>>>
    eliminate_one_route_per_day(const std::vector<std::vector<std::vector<std::string>>> &routes_by_day26,
                                const std::unordered_map<std::string, int> &service_min)
    {
        int days = (int)routes_by_day26.size();
        int veh26 = (int)routes_by_day26[0].size();
        int veh25 = veh26 - 1;
        std::vector<std::vector<std::vector<std::string>>> res(days, std::vector<std::vector<std::string>>(veh25));

        for (int d = 0; d < days; ++d)
        {
            // compute route service load
            std::vector<int> loads(veh26, 0);
            for (int r = 0; r < veh26; ++r)
                for (const auto &tid : routes_by_day26[d][r])
                    loads[r] += service_min.at(tid);

            // idx of lightest route to drop
            int drop = int(std::min_element(loads.begin(), loads.end()) - loads.begin());

            // copy remaining routes in order, skipping drop
            int wr = 0;
            for (int r = 0; r < veh26; ++r)
                if (r != drop)
                    res[d][wr++] = routes_by_day26[d][r];

            // append dropped tasks into current lightest destination route
            if (!routes_by_day26[d][drop].empty())
            {
                std::vector<int> kept_loads(veh25, 0);
                for (int r = 0; r < veh25; ++r)
                    for (const auto &tid : res[d][r])
                        kept_loads[r] += service_min.at(tid);

                int dst = int(std::min_element(kept_loads.begin(), kept_loads.end()) - kept_loads.begin());
                auto &dst_vec = res[d][dst];
                const auto &src_vec = routes_by_day26[d][drop];
                dst_vec.insert(dst_vec.end(), src_vec.begin(), src_vec.end());
            }
        }
        return res;
    }

    // ---------- Hungarian (min-cost) for square matrix ----------
    double hungarian_min_cost(const std::vector<std::vector<double>> &a)
    {
        const int n = (int)a.size();
        if (n == 0)
            return 0.0;
        const double INF = std::numeric_limits<double>::infinity();

        // 1-based arrays
        std::vector<double> u(n + 1, 0), v(n + 1, 0);
        std::vector<int> p(n + 1, 0), way(n + 1, 0);

        for (int i = 1; i <= n; ++i)
        {
            p[0] = i;
            int j0 = 0;
            std::vector<double> minv(n + 1, INF);
            std::vector<char> used(n + 1, false);
            do
            {
                used[j0] = true;
                int i0 = p[j0], j1 = 0;
                double delta = INF;
                for (int j = 1; j <= n; ++j)
                    if (!used[j])
                    {
                        double cur = a[i0 - 1][j - 1] - u[i0] - v[j];
                        if (cur < minv[j])
                        {
                            minv[j] = cur;
                            way[j] = j0;
                        }
                        if (minv[j] < delta)
                        {
                            delta = minv[j];
                            j1 = j;
                        }
                    }
                for (int j = 0; j <= n; ++j)
                {
                    if (used[j])
                    {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    }
                    else
                    {
                        minv[j] -= delta;
                    }
                }
                j0 = j1;
            } while (p[j0] != 0);
            do
            {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0 != 0);
        }

        // p[j] is the row matched to column j
        double value = 0.0;
        for (int j = 1; j <= n; ++j)
        {
            int i = p[j];
            if (i > 0)
                value += a[i - 1][j - 1];
        }
        return value;
    }

    // Build a Jaccard-cost matrix for two route sets of a given day, then Hungarian
    double day_hungarian_jaccard_cost(const std::vector<std::vector<std::string>> &A_routes,
                                      const std::vector<std::vector<std::string>> &B_routes)
    {
        const int RA = (int)A_routes.size();
        const int RB = (int)B_routes.size();
        const int N = std::max(RA, RB);
        if (N == 0)
            return 0.0;

        auto jaccard_cost = [](const std::vector<std::string> &a,
                               const std::vector<std::string> &b) -> double
        {
            if (a.empty() && b.empty())
                return 0.0; // identical empties → cost 0
            std::unordered_set<std::string> sa(a.begin(), a.end());
            std::unordered_set<std::string> sb(b.begin(), b.end());
            size_t inter = 0;
            // iterate over smaller set for speed
            if (sa.size() < sb.size())
            {
                for (const auto &x : sa)
                    if (sb.count(x))
                        ++inter;
            }
            else
            {
                for (const auto &x : sb)
                    if (sa.count(x))
                        ++inter;
            }
            size_t uni = sa.size() + sb.size() - inter;
            double jac = (uni == 0) ? 1.0 : (double)inter / (double)uni; // if both empty: similarity 1
            return 1.0 - jac;                                            // cost = 1 - similarity
        };

        // Build NxN cost (pad with cost=1 for dummies)
        std::vector<std::vector<double>> C(N, std::vector<double>(N, 1.0));
        for (int i = 0; i < RA; ++i)
            for (int j = 0; j < RB; ++j)
                C[i][j] = jaccard_cost(A_routes[i], B_routes[j]);

        const double total = hungarian_min_cost(C);
        return total / double(N); // average per matched pair (incl. dummies)
    }

} // anon namespace

// ---------------- API ----------------
namespace fsmm
{

    // Diversity metric: Hungarian-matched Jaccard per day, averaged across days.
    double assignment_distance(const SolSummary &a, const SolSummary &b)
    {
        const int DA = (int)a.routes_by_day.size();
        const int DB = (int)b.routes_by_day.size();
        if (DA == 0 && DB == 0)
            return 0.0;
        const int D = std::max(DA, DB);

        double sum = 0.0;
        for (int d = 0; d < D; ++d)
        {
            const auto &Ar = (d < DA) ? a.routes_by_day[d] : std::vector<std::vector<std::string>>{};
            const auto &Br = (d < DB) ? b.routes_by_day[d] : std::vector<std::vector<std::string>>{};
            sum += day_hungarian_jaccard_cost(Ar, Br);
        }
        return sum / double(D);
    }

    std::vector<int> select_diverse_indices(const std::vector<SolSummary> &sols,
                                            double distance_threshold)
    {
        // indices sorted by (overtime, travel)
        std::vector<int> idx(sols.size());
        std::iota(idx.begin(), idx.end(), 0);
        std::sort(idx.begin(), idx.end(), [&](int i, int j)
                  {
    if (sols[i].overtime_minutes != sols[j].overtime_minutes)
      return sols[i].overtime_minutes < sols[j].overtime_minutes;
    return sols[i].travel_minutes < sols[j].travel_minutes; });

        std::vector<int> keep;
        for (int id : idx)
        {
            bool far_enough = true;
            for (int kept : keep)
            {
                if (assignment_distance(sols[id], sols[kept]) < distance_threshold)
                {
                    far_enough = false;
                    break;
                }
            }
            if (far_enough)
                keep.push_back(id);
        }
        return keep;
    }

    std::vector<json> build_seeds(const json &tasks,
                                  const json &matrix_seconds,
                                  const json &base_config,
                                  const Config &mmcfg)
    {
        const int route_limit_minutes = base_config.value("MAX_HOURS_PER_DAY", 9) * 60;
        std::unordered_set<std::string> all_tasks = all_task_ids_from_tasks(tasks);
        std::unordered_map<std::string, int> task2client = task_client_map(tasks);
        std::unordered_map<std::string, int> service_min = task_service_minutes(tasks);

        // minutes matrix
        std::vector<std::vector<int>> M;
        M.reserve(matrix_seconds.size());
        for (const auto &row : matrix_seconds)
        {
            std::vector<int> r = row.get<std::vector<int>>();
            for (int &x : r)
                x = x / 60;
            M.push_back(std::move(r));
        }

        if (!sanity_check_M_vs_tasks(task2client, M))
        {
            throw std::runtime_error("distance/time matrix is incompatible with task client ids");
        }

        const std::string min_date = find_min_date(tasks);

        // 1) relaxed runs (26 vehicles)

        // Single-run worker (returns nothing when dropped-tasks or failure)
        auto run_one = [&](int i) -> std::optional<SolSummary>
        {
            try
            {
                first_schedule_solver::SolveOptions opts;
                opts.vehicles_per_day = mmcfg.vehicles_relaxed;
                opts.time_limit_seconds = mmcfg.time_limit_relaxed_s;
                opts.lns_time_limit_seconds = 3;
                opts.random_seed = (mmcfg.random_seed_base == 0 ? (12345 + i) : (mmcfg.random_seed_base + i));
                opts.num_solutions_to_collect = 1;
                opts.log_search = mmcfg.verbose;
                opts.enable_operator_bundle = true;

                opts.overtime_cap_minutes = 0;
                opts.overtime_soft_coeff = 0;

                // diversity knobs
                opts.shuffle_task_nodes = true;
                opts.shuffle_seed = opts.random_seed;
                opts.cost_noise_magnitude = 1;
                opts.cost_noise_seed = opts.random_seed;

                opts.first_solution_strategy = operations_research::FirstSolutionStrategy::PATH_CHEAPEST_ARC;

                json sol = first_schedule_solver::solve_problem(tasks, matrix_seconds, base_config, opts);

                auto rbd = extract_routes_by_day(sol);
                if (has_dropped_tasks(rbd, all_tasks))
                {
                    if (mmcfg.verbose)
                        std::cerr << "[skip] run#" << i << " drops tasks\n";
                    return std::nullopt;
                }

                auto sig = assignment_signature(rbd);
                auto met = compute_metrics(rbd, task2client, service_min, M, route_limit_minutes);

                SolSummary sum;
                sum.solution = std::move(sol);
                sum.routes_by_day = std::move(rbd);
                sum.signature = std::move(sig);
                sum.travel_minutes = met.travel;
                sum.overtime_minutes = met.overtime;
                sum.vehicles_per_day = mmcfg.vehicles_relaxed;
                sum.days = (int)sum.routes_by_day.size();
                sum.source_seed = opts.random_seed;
                return sum;
            }
            catch (const std::exception &e)
            {
                if (mmcfg.verbose)
                    std::cerr << "[err] run#" << i << ": " << e.what() << "\n";
                return std::nullopt;
            }
        };

        // --- batched parallel execution (chunks of 8) ---
        const int max_concurrency = 8; // tune if you like
        std::vector<SolSummary> inc26;
        inc26.reserve(mmcfg.runs);

        for (int base = 0; base < mmcfg.runs; base += max_concurrency)
        {
            int batch_end = std::min(mmcfg.runs, base + max_concurrency);

            std::vector<std::future<std::optional<SolSummary>>> futs;
            futs.reserve(batch_end - base);

            // launch batch
            for (int i = base; i < batch_end; ++i)
            {
                futs.emplace_back(std::async(std::launch::async, run_one, i));
            }

            // collect batch
            for (int j = 0; j < (int)futs.size(); ++j)
            {
                auto res = futs[j].get();
                if (res)
                    inc26.push_back(std::move(*res));
            }
        }

        // (Optional) deterministic post-order: sort by signature, or (travel, overtime), etc.
        std::sort(inc26.begin(), inc26.end(), [](const SolSummary &a, const SolSummary &b)
                  {
            if (a.travel_minutes != b.travel_minutes) return a.travel_minutes < b.travel_minutes;
            if (a.overtime_minutes != b.overtime_minutes) return a.overtime_minutes < b.overtime_minutes;
            return a.signature < b.signature; });

        if (inc26.empty())
        {
            std::cout << "collected_26=0\n";
            return {};
        }

        // === Adaptive overtime weight (one-liner derived) ===
        long long tot_service = 0, tot_travel = 0;
        for (const auto &s : inc26)
        {
            for (const auto &day : s.routes_by_day)
                for (const auto &r : day)
                {
                    for (const auto &tid : r)
                        tot_service += service_min[tid];
                    tot_travel += route_travel_minutes(r, task2client, M);
                }
        }
        double service_share = (tot_service + tot_travel) ? (double)tot_service / (double)(tot_service + tot_travel) : 0.7;
        g_W_OVERTIME = clampd(1.5 + 2.0 * service_share, 1.0, 4.0);
        if (mmcfg.verbose)
            std::cout << "adaptive_W_OVERTIME=" << g_W_OVERTIME
                      << " (service_share=" << service_share << ")\n";

        // 2) fast dedup (by signature + diversity)
        std::sort(inc26.begin(), inc26.end(), [](const SolSummary &a, const SolSummary &b)
                  {
        if (a.overtime_minutes != b.overtime_minutes) return a.overtime_minutes < b.overtime_minutes;
        return a.travel_minutes < b.travel_minutes; });

        // unique after signature
        std::unordered_set<std::string> all_sigs;
        for (const auto &s : inc26)
            all_sigs.insert(s.signature);

        std::vector<SolSummary> kept26;
        std::unordered_set<std::string> seen_sig;
        for (const auto &s : inc26)
        {
            if (!seen_sig.insert(s.signature).second)
                continue;
            bool far = true;
            for (const auto &k : kept26)
            {
                if (assignment_distance(s, k) < mmcfg.diversity_threshold)
                {
                    far = false;
                    break;
                }
            }
            if (far)
                kept26.push_back(s);
            if ((int)kept26.size() >= 12)
                break;
        }

        std::cout << "collected_26=" << inc26.size()
                  << " unique_after_sig=" << all_sigs.size()
                  << " kept26_after_diversity=" << kept26.size() << "\n";

        if (kept26.empty())
            return {};

        // 3) eliminate 26→25 with fallback ladder; 4) warm-start polish (stricter acceptance)
        std::vector<SolSummary> seeds25_base;
        const int kElimOvertimeCapMinutes = 45; // elimination cap
        const int kPolishAcceptOverCapMin = 20; // stricter acceptance after polish

        // Baseline task count for sanity
        const size_t baseline_tasks = count_tasks_in_rbd(kept26[0].routes_by_day);

        int elim_success = 0;
        for (const auto &s26 : kept26)
        {
            // Sanity: equal task count before elimination
            if (count_tasks_in_rbd(s26.routes_by_day) != baseline_tasks)
            {
                if (mmcfg.verbose)
                    std::cerr << "[skip] seed#" << s26.source_seed << " task-count mismatch pre-elim\n";
                continue;
            }

            std::vector<std::vector<std::vector<std::string>>> rbd25;
            bool ok_elim = eliminate_one_route_per_day_regret(
                s26.routes_by_day, service_min, task2client, M,
                route_limit_minutes, kElimOvertimeCapMinutes, rbd25);
            if (!ok_elim)
            {
                if (mmcfg.verbose)
                    std::cerr << "[skip] elimination failed for seed#" << s26.source_seed << "\n";
                continue;
            }
            ++elim_success;

            // Sanity: same tasks after elimination
            if (count_tasks_in_rbd(rbd25) != baseline_tasks)
            {
                if (mmcfg.verbose)
                    std::cerr << "[skip] post-elim task-count mismatch for seed#" << s26.source_seed << "\n";
                continue;
            }
            if (!check_route_counts(rbd25, mmcfg.vehicles_target))
            {
                if (mmcfg.verbose)
                    std::cerr << "[skip] post-elim route count mismatch for seed#" << s26.source_seed << "\n";
                continue;
            }

            // fixed-day tasks (freeze day choices) for polish
            json fixed_tasks = build_fixed_day_tasks(tasks, rbd25, min_date);

            // warm-start polish at 25
            first_schedule_solver::SolveOptions wopts;
            wopts.vehicles_per_day = mmcfg.vehicles_target;
            wopts.time_limit_seconds = mmcfg.time_limit_polish_s;
            wopts.lns_time_limit_seconds = 3;
            wopts.num_solutions_to_collect = 1;
            wopts.log_search = mmcfg.verbose;
            wopts.enable_operator_bundle = true;

            // POLISH: allow small overtime to avoid drops, but keep it tight
            wopts.overtime_cap_minutes = kPolishAcceptOverCapMin; // e.g., 15–20
            wopts.overtime_soft_coeff = 2000;                     // tune; linear penalty per minute

            // For polish, LCI usually works best:
            wopts.first_solution_strategy = operations_research::FirstSolutionStrategy::LOCAL_CHEAPEST_INSERTION;

            // (If solver later supports it, also set wopts.overtime_cap_minutes = kPolishAcceptOverCapMin)

            json polished = first_schedule_solver::solve_problem(fixed_tasks, matrix_seconds, base_config, wopts);
            auto rbd_final = extract_routes_by_day(polished);

            // Sanity: full coverage & per-day route count
            if (has_dropped_tasks(rbd_final, all_tasks))
            {
                if (mmcfg.verbose)
                    std::cerr << "[skip] polished dropped tasks seed#" << s26.source_seed << "\n";
                continue;
            }
            if (!check_route_counts(rbd_final, mmcfg.vehicles_target))
            {
                if (mmcfg.verbose)
                    std::cerr << "[skip] polished route count mismatch seed#" << s26.source_seed << "\n";
                continue;
            }

            // Stricter acceptance: no route exceeding limit + 20
            int max_over = max_overage_minutes(rbd_final, task2client, service_min, M, route_limit_minutes);
            if (max_over > kPolishAcceptOverCapMin)
            {
                if (mmcfg.verbose)
                    std::cerr << "[skip] polished max_over=" << max_over
                              << " > " << kPolishAcceptOverCapMin
                              << " seed#" << s26.source_seed << "\n";
                continue;
            }

            auto sig = assignment_signature(rbd_final);
            auto met = compute_metrics(rbd_final, task2client, service_min, M, route_limit_minutes);
            auto [avg_load, max_load] = load_stats_minutes(rbd_final, task2client, service_min, M);

            SolSummary s25;
            s25.solution = std::move(polished);
            s25.routes_by_day = std::move(rbd_final);
            s25.signature = std::move(sig);
            s25.travel_minutes = met.travel;
            s25.overtime_minutes = met.overtime;
            s25.vehicles_per_day = mmcfg.vehicles_target;
            s25.days = (int)s25.routes_by_day.size();
            s25.source_seed = s26.source_seed;
            s25.avg_load_minutes = avg_load;
            s25.max_load_minutes = max_load;

            seeds25_base.push_back(std::move(s25));
        }

        if (seeds25_base.empty())
        {
            std::cout << "elim_success_count=" << elim_success << " final_25_count=0\n";
            return {};
        }

        // Final greedy selection by diversity
        auto keep_idx = select_diverse_indices(seeds25_base, mmcfg.diversity_threshold);

        // Pairwise distance stats on final set (min/median)
        double mind = 1.0, mediand = 0.0;
        if (keep_idx.size() >= 2)
        {
            std::vector<double> dists;
            for (size_t i = 0; i < keep_idx.size(); ++i)
                for (size_t j = i + 1; j < keep_idx.size(); ++j)
                {
                    double d = assignment_distance(seeds25_base[keep_idx[i]], seeds25_base[keep_idx[j]]);
                    dists.push_back(d);
                    mind = std::min(mind, d);
                }
            std::sort(dists.begin(), dists.end());
            mediand = dists[dists.size() / 2];
        }

        std::cout << "elim_success_count=" << elim_success
                  << " final_25_count=" << keep_idx.size()
                  << " diversity_min=" << (keep_idx.size() >= 2 ? mind : 0.0)
                  << " diversity_median=" << (keep_idx.size() >= 2 ? mediand : 0.0) << "\n";

        // Return solutions with metadata embedded for diagnostics
        std::vector<json> out;
        out.reserve(keep_idx.size());
        for (int idx : keep_idx)
        {
            const auto &s = seeds25_base[idx];
            json wrapped = {
                {"meta", {{"source_seed", s.source_seed}, {"overtime_min", s.overtime_minutes}, {"travel_min", s.travel_minutes}, {"avg_load_min", s.avg_load_minutes}, {"max_load_min", s.max_load_minutes}, {"signature", s.signature}}},
                {"solution", s.solution}};
            out.push_back(std::move(wrapped));
        }
        return out;
    }

} // namespace fsmm
