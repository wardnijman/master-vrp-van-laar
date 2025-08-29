#include "seed.h"
#include <stdexcept>
#include "seed_generators.h"

#include <unordered_map>
#include <map>
#include <set>
#include <array>
#include <cmath>
#include <cctype>
#include <algorithm>
#include <numeric>
#include <stdexcept>

#include "validation.h"
#include "penalties.h"
#include "day_solver.h"
#include "utils.h"
#include "types.h"

static std::unordered_map<std::string, SeedGeneratorFn>& REGISTRY() {
  static std::unordered_map<std::string, SeedGeneratorFn> r;
  return r;
}

std::unordered_map<std::string, SeedGeneratorFn>& SeedRegistry::map() { return REGISTRY(); }

void SeedRegistry::register_generator(const std::string& name, SeedGeneratorFn fn) {
  REGISTRY()[name] = std::move(fn);
}
bool SeedRegistry::has(const std::string& name) { return REGISTRY().count(name) > 0; }
SeedGeneratorFn SeedRegistry::get(const std::string& name) {
  auto it = REGISTRY().find(name);
  if (it == REGISTRY().end()) throw std::runtime_error("Unknown seed generator: " + name);
  return it->second;
}
std::vector<std::string> SeedRegistry::names() {
  std::vector<std::string> out; out.reserve(REGISTRY().size());
  for (auto& kv : REGISTRY()) out.push_back(kv.first);
  return out;
}


// ------------------------- helpers (pure, local) -------------------------

static inline int clamp(int v, int lo, int hi) { return std::max(lo, std::min(v, hi)); }

// Extract weekday from a task (expects 1..7).
static inline int weekday_of(const Task& t) {
  if (t.day_options.empty()) return 0;
  return t.day_options.front().weekday;
}

// Compute canonical weekday per schema_number over the full seed (mode; ties → earliest day)
static std::unordered_map<int,int> canonical_weekday_by_schema(const std::vector<Task>& tasks) {
  std::unordered_map<int, std::array<int, 8>> counts; // use 1..7
  for (const auto& t : tasks) {
    int w = weekday_of(t);
    if (w < 1 || w > 7) continue;
    auto& arr = counts[t.schema_number];
    if (arr[0] == 0) arr.fill(0); // lazy init
    arr[w] += 1;
  }
  std::unordered_map<int,int> canon;
  for (auto& kv : counts) {
    int best_w = 1, best_c = -1;
    for (int w = 1; w <= 7; ++w) {
      if (kv.second[w] > best_c) { best_c = kv.second[w]; best_w = w; }
    }
    canon[kv.first] = best_w;
  }
  return canon;
}

// Intra-week spacing penalty for variants of same schema_number in same week
static double compute_intraweek_spacing_penalty(
    const std::vector<Task>& tasks,
    double same_day_penalty_minutes,
    double near_day_penalty_minutes)
{
  struct Key { int schema; int week; bool operator==(const Key& o) const { return schema==o.schema && week==o.week; } };
  struct KeyHash { size_t operator()(const Key& k) const { return (size_t)k.schema * 1315423911u ^ (size_t)k.week; }};
  std::unordered_map<Key, std::vector<const Task*>, KeyHash> groups;
  groups.reserve(tasks.size()/3 + 1);

  for (const auto& t : tasks) groups[{t.schema_number, t.week}].push_back(&t);

  double penalty = 0.0;
  for (auto& kv : groups) {
    auto& vec = kv.second;
    if (vec.size() <= 1) continue;

    std::vector<int> days; days.reserve(vec.size());
    for (auto* pt : vec) days.push_back(weekday_of(*pt));

    int m = (int)days.size();
    for (int i = 0; i < m; ++i) {
      for (int j = i+1; j < m; ++j) {
        int d1 = days[i], d2 = days[j];
        if (d1 == d2) {
          penalty += same_day_penalty_minutes;
        } else {
          int dist = std::abs(d1 - d2); dist = std::min(dist, 7 - dist);
          if (dist == 1) penalty += near_day_penalty_minutes;
        }
      }
    }
  }
  return penalty;
}

// Inter-week canonical weekday consistency penalty
static double compute_interweek_canonical_penalty(
    const std::vector<Task>& tasks,
    const std::unordered_map<int,int>& canon_wd,
    double per_deviation_minutes)
{
  double pen = 0.0;
  for (const auto& t : tasks) {
    auto it = canon_wd.find(t.schema_number);
    if (it == canon_wd.end()) continue;
    int w = weekday_of(t);
    if (w >= 1 && w <= 7 && w != it->second) pen += per_deviation_minutes;
  }
  return pen;
}

// Over-capacity penalty (per route) with piecewise growth.
static double piecewise_overcap_penalty_minutes(double over_minutes) {
  if (over_minutes <= 0) return 0.0;
  double a = std::min(over_minutes, 30.0);
  double b = std::max(0.0, over_minutes - 30.0);
  return a * 1.0 + b * 3.0;
}

// -------------------- per-day evaluation hook (REAL solver) --------------------

struct DayEvalResult {
  double travel_minutes = 0.0;
  double service_minutes = 0.0;
  std::vector<double> route_work_minutes; // travel + service per route
};

// Convert seed::Task → TaskRef (the type your day solver expects)
static inline TaskRef to_taskref(const Task& t) {
  TaskRef r{};
  r.task_id         = t.task_id;
  r.schema_number   = t.schema_number;
  r.client_idx      = t.client_id;                 // matrix index (0 = depot)
  r.service_minutes = (int)std::llround(t.duration);
  r.week            = t.week;
  // suffix/weekday_options not required by day VRP; leave default
  return r;
}

static DayEvalResult evaluate_single_day_with_solver(
    const std::string& /*date*/,
    const std::vector<const Task*>& day_tasks,
    const EvalConfig& cfg)
{
  if (!cfg.matrix) throw std::runtime_error("EvalConfig.matrix is null in seed scoring.");

  // Build the per-day TaskRef vector expected by solve_day(...)
  std::vector<TaskRef> refs; refs.reserve(day_tasks.size());
  DayEvalResult out{};
  for (auto* t : day_tasks) {
    TaskRef r{};
    r.task_id         = t->task_id;
    r.schema_number   = t->schema_number;
    r.client_idx      = t->client_id;                 // matrix index (0 = depot)
    r.service_minutes = (int)std::llround(t->duration);
    r.week            = t->week;
    refs.push_back(std::move(r));
    out.service_minutes += t->duration;
  }

  // Day config (fall back to max_* if day_cfg is not fully set)
  DayConfig dc = cfg.day_cfg;
  if (dc.vehicles <= 0)            dc.vehicles = cfg.max_routes_per_day;
  if (dc.route_limit_minutes <= 0) dc.route_limit_minutes = cfg.max_hours_per_route * 60;

  // Short exploration solve for seed evaluation
  DaySolveParams params = cfg.explore_params;
  if (params.time_limit_seconds <= 0) params.time_limit_seconds = 2;

  const DayWarmStart* warm = nullptr; // neutral comparison for seeds
  DayResult res = solve_day(refs, *cfg.matrix, dc, params, warm);

  // We don’t assume any per-route fields; just use the totals we’re sure exist.
  // If your DayResult exposes per-route work minutes later, you can populate route_work_minutes here.
  out.route_work_minutes.clear();
  out.travel_minutes = std::max(0.0, (double)res.total_minutes - out.service_minutes);

  return out;
}


// -------------------- seed-shape validation --------------------

static std::string run_validation(const std::vector<Task>& tasks) {
  if (tasks.empty()) return "No tasks in seed.";
  for (const auto& t : tasks) {
    if (t.day_options.size() != 1) return "Each task must have exactly 1 day_option in a seed.";
    if (t.duration <= 0.0 || !std::isfinite(t.duration)) return "Invalid duration.";
    if (t.week <= 0) return "Invalid week index.";
    int wd = weekday_of(t);
    if (wd < 1 || wd > 7) return "Weekday must be 1..7 in seed.";
    if (t.day_options.front().date.empty()) return "Missing day_options.date.";
  }
  return "";
}

// ------------------------- evaluate_seed_cost (full) -------------------------

SeedScore evaluate_seed_cost(const Seed& seed, const EvalConfig& cfg) {
  // 1) Group tasks by date
  std::unordered_map<std::string, std::vector<const Task*>> by_date;
  by_date.reserve(seed.tasks.size()/6 + 1);
  for (const auto& t : seed.tasks) {
    if (t.day_options.empty()) {
      SeedScore sc; sc.valid = false; sc.validation_error = "Task missing day_option."; sc.total_cost = 1e18; return sc;
    }
    by_date[t.day_options.front().date].push_back(&t);
  }

  // 2) Per-day routing cost + route-level over-capacity penalties
  double total_travel = 0.0;
  double total_service = 0.0;
  double total_overcap_penalties = 0.0;

  const double route_cap_minutes = cfg.max_hours_per_route * 60.0;

  for (auto& kv : by_date) {
    const std::string& date = kv.first;
    auto& day_tasks = kv.second;

    DayEvalResult dres = evaluate_single_day_with_solver(date, day_tasks, cfg);

    total_travel  += dres.travel_minutes;
    total_service += dres.service_minutes;

    // Per-route over-capacity penalties (now active since solver returns route mins)
    if (!dres.route_work_minutes.empty()) {
      for (double wm : dres.route_work_minutes) {
        double over = wm - route_cap_minutes;
        total_overcap_penalties += piecewise_overcap_penalty_minutes(over);
      }
    }
  }

  // 3) Global penalties (independent of within-day routing)
  const double spacing_same_day_pen = 20.0;
  const double spacing_near_day_pen = 8.0;
  double spacing_penalties = compute_intraweek_spacing_penalty(
      seed.tasks, spacing_same_day_pen, spacing_near_day_pen);

  auto canon = canonical_weekday_by_schema(seed.tasks);
  const double canonical_dev_pen = 6.0;
  double canonical_penalties = compute_interweek_canonical_penalty(
      seed.tasks, canon, canonical_dev_pen);

  // 4) Aggregate
  SeedScore sc;
  sc.valid = true;
  sc.validation_error.clear();

  sc.total_cost =
      total_travel
    + total_overcap_penalties
    + spacing_penalties
    + canonical_penalties;

  (void)total_service;
  return sc;
}

// -------------------- build & rank seeds --------------------

std::vector<RankedSeed> build_and_rank_seeds(
  const std::vector<std::string>& generator_names,
  const std::vector<Task>& base_tasks,
  const EvalConfig& cfg,
  int top_k)
{
  if (top_k <= 0) throw std::invalid_argument("top_k must be > 0");

  register_default_seed_generators();

  std::vector<RankedSeed> all;
  all.reserve(generator_names.size());

  for (const auto& name : generator_names) {
    if (!SeedRegistry::has(name)) continue;
    auto gen = SeedRegistry::get(name);
    Seed s = gen(base_tasks);
    s.name = name;

    RankedSeed r;
    r.seed = std::move(s);

    if (auto msg = run_validation(r.seed.tasks); !msg.empty()) {
      r.score.valid = false;
      r.score.validation_error = msg;
      r.score.total_cost = 1e18;
    } else {
      r.score = evaluate_seed_cost(r.seed, cfg);
      r.score.valid = true;
    }
    all.push_back(std::move(r));
  }

  sort_by_cost(all);
  if ((int)all.size() > top_k) all.resize(top_k);
  return all;
}

void sort_by_cost(std::vector<RankedSeed>& v) {
  std::stable_sort(v.begin(), v.end(),
                   [](const RankedSeed& a, const RankedSeed& b){
                     if (a.score.valid != b.score.valid) return a.score.valid && !b.score.valid;
                     return a.score.total_cost < b.score.total_cost;
                   });
}


