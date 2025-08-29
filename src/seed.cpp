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
#include <random>

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


// ---------- Diversity utilities ----------

// Build maps: task_id -> chosen weekday/date
static std::unordered_map<std::string,int> map_weekday(const Seed& s) {
  std::unordered_map<std::string,int> m; m.reserve(s.tasks.size());
  for (auto& t : s.tasks) if (!t.day_options.empty()) m[t.task_id]=t.day_options.front().weekday;
  return m;
}
static std::unordered_map<std::string,std::string> map_date(const Seed& s) {
  std::unordered_map<std::string,std::string> m; m.reserve(s.tasks.size());
  for (auto& t : s.tasks) if (!t.day_options.empty()) m[t.task_id]=t.day_options.front().date;
  return m;
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

std::string run_validation(const std::vector<Task>& tasks) {
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

// Hamming distance over chosen weekday (normalized by |intersection of tasks|)
static double weekday_hamming(const Seed& a, const Seed& b) {
  auto ma = map_weekday(a), mb = map_weekday(b);
  size_t agree=0, total=0;
  for (auto& kv : ma) {
    auto it = mb.find(kv.first);
    if (it==mb.end()) continue;
    total++;
    if (it->second == kv.second) agree++;
  }
  if (total==0) return 0.0;
  return 1.0 - (double)agree / (double)total;
}

// Per-date Jaccard of client sets, averaged
static double perdate_jaccard(const Seed& a, const Seed& b) {
  auto ada = map_date(a), bda = map_date(b);
  // invert to date -> set<client>
  std::unordered_map<std::string, std::set<int>> A,B;
  for (auto& t : a.tasks) if (!t.day_options.empty()) A[t.day_options.front().date].insert(t.client_id);
  for (auto& t : b.tasks) if (!t.day_options.empty()) B[t.day_options.front().date].insert(t.client_id);

  // union of dates
  std::set<std::string> dates;
  for (auto& kv : A) dates.insert(kv.first);
  for (auto& kv : B) dates.insert(kv.first);

  double sum=0.0; int cnt=0;
  for (auto& d : dates) {
    const auto& Sa = A[d];
    const auto& Sb = B[d];
    if (Sa.empty() && Sb.empty()) continue;
    // Jaccard distance = 1 - |∩|/|∪|
    std::vector<int> inter, uni;
    std::set_intersection(Sa.begin(),Sa.end(), Sb.begin(),Sb.end(), std::back_inserter(inter));
    std::set_union       (Sa.begin(),Sa.end(), Sb.begin(),Sb.end(), std::back_inserter(uni));
    double jd = 1.0 - (uni.empty() ? 0.0 : (double)inter.size()/(double)uni.size());
    sum += jd; cnt++;
  }
  return cnt ? (sum / cnt) : 0.0;
}

// Final distance: convex combo
static double seed_distance(const Seed& a, const Seed& b) {
  const double w_ham = 0.7, w_jac = 0.3;
  return w_ham * weekday_hamming(a,b) + w_jac * perdate_jaccard(a,b);
}

// Greedy max–min selection: pick one best (low cost), then repeatedly add the seed
// that maximizes distance to the current selected set (min distance to any selected).
static std::vector<RankedSeed> pick_diverse(const std::vector<RankedSeed>& in, int k) {
  if ((int)in.size() <= k) return in;
  // start from the best cost
  std::vector<RankedSeed> cand = in;
  std::vector<RankedSeed> out; out.reserve(k);
  out.push_back(cand.front());

  while ((int)out.size() < k) {
    int best_i = -1; double best_score=-1.0;
    for (int i=1;i<(int)cand.size();++i) {
      const Seed& s = cand[i].seed;
      double mind = 1e9;
      for (auto& rsel : out) {
        mind = std::min(mind, seed_distance(s, rsel.seed));
      }
      if (mind > best_score) { best_score = mind; best_i = i; }
    }
    if (best_i < 0) break;
    out.push_back(cand[best_i]);
    cand.erase(cand.begin() + best_i);
  }
  return out;
}

// New: build many candidates (with slight randomization), keep top_by_cost per generator,
// then pick a final diverse_k across all.
std::vector<RankedSeed> build_rank_and_pick_diverse(
  const std::vector<std::string>& generator_names,
  const std::vector<Task>& base_tasks,
  const EvalConfig& cfg,
  int per_generator_trials,
  int keep_top_by_cost_per_gen,
  int diverse_k,
  int rng_seed)
{
  register_default_seed_generators();

  std::mt19937 rng(rng_seed);
  std::vector<RankedSeed> all;

  for (const auto& name : generator_names) {
    if (!SeedRegistry::has(name)) continue;
    auto gen = SeedRegistry::get(name);

    std::vector<RankedSeed> bucket;

    for (int t=0; t<per_generator_trials; ++t) {
      // light randomization: shuffle a copy for generators that consider order
      std::vector<Task> shuffled = base_tasks;
      std::shuffle(shuffled.begin(), shuffled.end(), rng);

      Seed s = gen(shuffled);
      s.name = name;

      RankedSeed r; r.seed = std::move(s);
      if (auto msg = run_validation(r.seed.tasks); !msg.empty()) {
        r.score.valid = false; r.score.validation_error = msg; r.score.total_cost = 1e18;
      } else {
        r.score = evaluate_seed_cost(r.seed, cfg);
      }
      bucket.push_back(std::move(r));
    }

    sort_by_cost(bucket);
    if ((int)bucket.size() > keep_top_by_cost_per_gen) bucket.resize(keep_top_by_cost_per_gen);
    all.insert(all.end(), bucket.begin(), bucket.end());
  }

  sort_by_cost(all);
  // Now pick a diverse subset across all
  if (diverse_k > 0 && (int)all.size() > diverse_k) {
    return pick_diverse(all, diverse_k);
  }
  return all;
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





