#include "seed_generators.h"
#include <algorithm>
#include <unordered_map>
#include <random>
#include <map>
#include <set>
#include <numeric>
#include <climits>
#include "utils.h"
// ---------------- TimeMatrix glue ----------------
#include "types.h"

static const TimeMatrix *G_M = nullptr;
void seed_generators_set_matrix(const TimeMatrix *M) { G_M = M; }
static inline int D(int a, int b) { return (G_M ? G_M->at(a, b) : 0); } // minutes

// --- small helpers ----------------------------------------------------------
struct DayBucket
{
  // Index-based to avoid string compares in hot path
  int date_idx = -1;
  int medoid_client = -1;   // -1 = not set
  std::vector<int> clients; // client ids assigned to this date
  int load_minutes = 0;     // sum of task durations (service)
};

// --- ISO weekday utilities -------------------------------------------------
// Sakamoto's algorithm: 0=Sunday..6=Saturday
static int dow_sakamoto(int y, int m, int d)
{
  static const int t[12] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  if (m < 3)
    y -= 1;
  return (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;
}

// Parse "YYYY-MM-DD" → ISO weekday 1..7 (Mon=1..Sun=7)
static int iso_weekday_from_date(const std::string &ymd)
{
  // Minimal, fast parser (assumes valid YYYY-MM-DD)
  int y = 0, m = 0, d = 0;
  // Safer than std::stoi substr in tight loops
  // Format: 0123456789
  //         YYYY-MM-DD
  if (ymd.size() >= 10)
  {
    y = (ymd[0] - '0') * 1000 + (ymd[1] - '0') * 100 + (ymd[2] - '0') * 10 + (ymd[3] - '0');
    m = (ymd[5] - '0') * 10 + (ymd[6] - '0');
    d = (ymd[8] - '0') * 10 + (ymd[9] - '0');
  }
  int dow = dow_sakamoto(y, m, d); // 0=Sun..6=Sat
  return (dow == 0) ? 7 : dow;     // ISO 1..7
}

// Normalize a DayOption's weekday to ISO 1..7 using its date
static Task::DayOption normalized_option(const Task::DayOption &in)
{
  Task::DayOption out = in;
  out.weekday = iso_weekday_from_date(in.date);
  return out;
}

// Normalize-by-date: pick the option with this date and write ISO weekday (1..7)
static Task choose_option_iso_by_date(const Task &t, const std::string &dateToKeep)
{
  Task out = t;
  out.day_options.clear();
  for (const auto &o : t.day_options)
  {
    if (o.date == dateToKeep)
    {
      // reuse your existing normalized_option(...) → sets 1..7 from date
      out.day_options.push_back(normalized_option(o));
      return out;
    }
  }
  // fallback: normalize the first one
  out.day_options.push_back(normalized_option(t.day_options.front()));
  return out;
}

static int recompute_medoid(const std::vector<int> &clients)
{
  if (clients.empty())
    return -1;
  int best = clients[0], best_sum = INT_MAX;
  for (int i : clients)
  {
    int s = 0;
    for (int j : clients)
      s += D(i, j);
    if (s < best_sum)
    {
      best_sum = s;
      best = i;
    }
  }
  return best;
}

// Incremental geo “cost” of adding client c to bucket b.
// Approximate Δ by distance to current medoid or depot, whichever smaller.
static inline int delta_geo_cost(const DayBucket &b, int client_id)
{
  if (b.clients.empty() || b.medoid_client < 0)
  {
    return D(0, client_id) + D(client_id, 0); // depot star
  }
  const int to_med = D(client_id, b.medoid_client);
  const int to_dep = D(0, client_id);
  return std::min(to_med, to_dep);
}

static int target_load_per_date_minutes(const std::vector<Task> &tasks,
                                        int n_dates)
{
  long long total = 0;
  for (const auto &t : tasks)
    total += (long long)std::llround(t.duration);
  const int nd = std::max(1, n_dates);
  return (int)std::max(1LL, total / nd);
}

static std::vector<std::string> collect_dates(const std::vector<Task> &tasks)
{
  std::set<std::string> s;
  for (const auto &t : tasks)
    for (const auto &o : t.day_options)
      if (!o.date.empty())
        s.insert(o.date);
  return std::vector<std::string>(s.begin(), s.end());
}

static Task choose_option_unchanged(const Task &t, const std::string &dateToKeep)
{
  Task out = t;
  out.day_options.clear();
  for (const auto &o : t.day_options)
    if (o.date == dateToKeep)
    {
      out.day_options.push_back(o);
      return out;
    }
  out.day_options.push_back(t.day_options.front()); // fallback
  return out;
}

// ---------------- geo_balanced (O(n·k)) -------------------------------------
Seed geo_balanced_seed(const std::vector<Task> &tasks)
{
  Seed s;
  s.name = "geo_balanced";
  s.tasks.reserve(tasks.size());
  if (!G_M)
    return passthrough_seed(tasks); // safety

  // Map date string -> compact index
  const auto dates = collect_dates(tasks);
  const int Dn = (int)dates.size();
  std::unordered_map<std::string, int> date2idx;
  date2idx.reserve(Dn * 2);
  for (int i = 0; i < Dn; ++i)
    date2idx[dates[i]] = i;

  // Buckets by date index
  std::vector<DayBucket> B(Dn);
  for (int i = 0; i < Dn; ++i)
  {
    B[i].date_idx = i;
  }

  const int target_load = target_load_per_date_minutes(tasks, Dn);
  const double beta = 0.8; // load penalty weight

  // Pre-sort: fixed first, then longer first
  std::vector<const Task *> order;
  order.reserve(tasks.size());
  for (auto &t : tasks)
    order.push_back(&t);
  std::stable_sort(order.begin(), order.end(),
                   [](const Task *a, const Task *b)
                   {
                     if (a->day_options.size() != b->day_options.size())
                       return a->day_options.size() < b->day_options.size();
                     return a->duration > b->duration;
                   });

  for (const Task *tp : order)
  {
    const Task &t = *tp;
    if (t.day_options.empty())
    {
      s.tasks.push_back(t);
      continue;
    }

    const Task::DayOption *best_o = nullptr;
    double best_score = 1e18;

    for (const auto &o : t.day_options)
    {
      const auto it = date2idx.find(o.date);
      if (it == date2idx.end())
        continue;
      auto &b = B[it->second];
      if (!b.clients.empty() && b.medoid_client < 0)
        b.medoid_client = recompute_medoid(b.clients);

      const int dgeo = delta_geo_cost(b, t.client_id);
      const double load_ratio = double(b.load_minutes) / std::max(1, target_load);
      const double score = dgeo * (1.0 + beta * load_ratio);
      if (score < best_score)
      {
        best_score = score;
        best_o = &o;
      }
    }

    if (!best_o)
      best_o = &t.day_options.front();
    auto &bsel = B[date2idx[best_o->date]];
    bsel.clients.push_back(t.client_id);
    bsel.load_minutes += (int)std::llround(t.duration);

    // Lazy medoid (recompute rarely)
    if (bsel.medoid_client < 0 || ((int)bsel.clients.size() & 127) == 0)
      bsel.medoid_client = recompute_medoid(bsel.clients);

    s.tasks.push_back(choose_option_iso_by_date(t, best_o->date));
  }

  // Final medoid refresh (optional)
  for (auto &b : B)
    if (!b.clients.empty())
      b.medoid_client = recompute_medoid(b.clients);

  return s;
}

// ---------------- geo_regret2 (heap, lazy updates) --------------------------
struct TaskView
{
  int idx; // index in `tasks`
  int client;
  int duration_min;       // rounded
  std::vector<int> opt_d; // allowed date indices
};

struct Key
{
  // Max-heap by regret2
  double regret2;
  int task_idx;
  // For lazy-stale detection
  int best_date; // snapshot when inserted
};

struct KeyCmp
{
  bool operator()(const Key &a, const Key &b) const
  {
    return a.regret2 < b.regret2; // max-heap
  }
};

Seed geo_regret2_seed(const std::vector<Task> &tasks)
{
  Seed s;
  s.name = "geo_regret2";
  s.tasks.reserve(tasks.size());
  if (!G_M)
    return passthrough_seed(tasks);

  // Date mapping
  const auto date_list = collect_dates(tasks);
  const int Dn = (int)date_list.size();
  std::unordered_map<std::string, int> date2idx;
  date2idx.reserve(Dn * 2);
  for (int i = 0; i < Dn; ++i)
    date2idx[date_list[i]] = i;

  // Buckets
  std::vector<DayBucket> B(Dn);
  for (int i = 0; i < Dn; ++i)
  {
    B[i].date_idx = i;
  }

  // Version per date for lazy invalidation
  std::vector<uint32_t> date_ver(Dn, 0);

  // Build task views
  std::vector<TaskView> V;
  V.reserve(tasks.size());
  V.reserve(tasks.size());
  for (int ti = 0; ti < (int)tasks.size(); ++ti)
  {
    const auto &t = tasks[ti];
    TaskView tv{ti, t.client_id, (int)std::llround(t.duration), {}};
    tv.opt_d.reserve(t.day_options.size());
    for (const auto &o : t.day_options)
    {
      const auto it = date2idx.find(o.date);
      if (it != date2idx.end())
        tv.opt_d.push_back(it->second);
    }
    V.push_back(std::move(tv));
  }

  const int target_load = target_load_per_date_minutes(tasks, Dn);
  const double beta = 0.8;   // load penalty on overloaded dates
  const double gamma = 0.05; // slight preference for inherently costly placements

  auto eval_task = [&](const TaskView &tv,
                       /*out*/ int &best_d, double &best_s, double &second_s)
  {
    best_d = -1;
    best_s = 1e18;
    second_s = 1e18;
    for (int d : tv.opt_d)
    {
      auto &b = B[d];
      if (!b.clients.empty() && b.medoid_client < 0)
        b.medoid_client = recompute_medoid(b.clients);
      const int dgeo = delta_geo_cost(b, tv.client);
      const double load_ratio = double(b.load_minutes) / std::max(1, target_load);
      const double score = dgeo * (1.0 + beta * load_ratio);
      if (score < best_s)
      {
        second_s = best_s;
        best_s = score;
        best_d = d;
      }
      else if (score < second_s)
      {
        second_s = score;
      }
    }
    if (tv.opt_d.empty())
    {
      best_d = -1;
      best_s = 0.0;
      second_s = 0.0;
    }
  };

  // Max-heap of regrets
  std::vector<Key> heap;
  heap.reserve(tasks.size());

  // Track best date + the date version we depended on when pushing.
  std::vector<int> cached_best_date(tasks.size(), -2);
  std::vector<uint32_t> cached_best_ver(tasks.size(), 0);

  // Initially compute all keys
  for (const auto &tv : V)
  {
    int bd;
    double s1, s2;
    eval_task(tv, bd, s1, s2);
    const double reg = (s2 - s1) + gamma * s1;
    heap.push_back({reg, tv.idx, bd});
    cached_best_date[tv.idx] = bd;
    cached_best_ver[tv.idx] = (bd >= 0 ? date_ver[bd] : 0);
  }
  std::make_heap(heap.begin(), heap.end(), KeyCmp{});

  // Assigned flag
  std::vector<char> taken(tasks.size(), 0);

  // Main loop
  while (!heap.empty())
  {
    std::pop_heap(heap.begin(), heap.end(), KeyCmp{});
    Key k = heap.back();
    heap.pop_back();

    const int ti = k.task_idx;
    if (taken[ti])
      continue;

    // If the date version changed since we pushed, recompute
    bool stale = false;
    const int bd_cached = cached_best_date[ti];
    if (bd_cached != k.best_date)
      stale = true;
    else if (bd_cached >= 0 && cached_best_ver[ti] != date_ver[bd_cached])
      stale = true;

    int bd;
    double s1, s2;
    if (stale)
    {
      eval_task(V[ti], bd, s1, s2);
      const double reg = (s2 - s1) + gamma * s1;
      heap.push_back({reg, ti, bd});
      std::push_heap(heap.begin(), heap.end(), KeyCmp{});
      cached_best_date[ti] = bd;
      cached_best_ver[ti] = (bd >= 0 ? date_ver[bd] : 0);
      continue;
    }
    else
    {
      // No stale: we need fresh scores as well to commit
      eval_task(V[ti], bd, s1, s2);
      // If best date changed due to ties/float noise, push updated and continue
      if (bd != k.best_date)
      {
        const double reg = (s2 - s1) + gamma * s1;
        heap.push_back({reg, ti, bd});
        std::push_heap(heap.begin(), heap.end(), KeyCmp{});
        cached_best_date[ti] = bd;
        cached_best_ver[ti] = (bd >= 0 ? date_ver[bd] : 0);
        continue;
      }
    }

    // Commit task ti to date bd (or fallback if none)
    const int commit_d = (bd >= 0) ? bd : (V[ti].opt_d.empty() ? -1 : V[ti].opt_d.front());
    if (commit_d < 0)
    {
      taken[ti] = 1;
      continue;
    }

    auto &bb = B[commit_d];
    bb.clients.push_back(V[ti].client);
    bb.load_minutes += V[ti].duration_min;

    // Occasional medoid refresh for this bucket
    if (bb.medoid_client < 0 || ((int)bb.clients.size() & 127) == 0)
      bb.medoid_client = recompute_medoid(bb.clients);

    // Invalidate tasks that depended on this date lazily
    ++date_ver[commit_d];

    taken[ti] = 1;

    // Emit chosen options
    const std::string &picked_date = date_list[commit_d];
    s.tasks.push_back(choose_option_iso_by_date(tasks[ti], picked_date));
  }

  // Final medoid refresh (optional)
  for (auto &b : B)
    if (!b.clients.empty())
      b.medoid_client = recompute_medoid(b.clients);

  return s;
}

// Helper: choose option and force ISO weekday
static Task choose_option(const Task &t, const Task::DayOption &opt)
{
  Task out = t;
  out.day_options.clear();
  out.day_options.push_back(normalized_option(opt));
  return out;
}

// (Kept but not used externally; safe)
static int wd_of(const Task &t)
{
  return t.day_options.empty()
             ? 0
             : iso_weekday_from_date(t.day_options.front().date);
}
static const std::string &date_of(const Task &t) { return t.day_options.front().date; }

void register_default_seed_generators()
{
  SeedRegistry::register_generator("passthrough", &passthrough_seed);
  SeedRegistry::register_generator("two_week_rollout", &two_week_rollout_seed);
  SeedRegistry::register_generator("capacity_balanced", &capacity_balanced_seed);
  SeedRegistry::register_generator("sweep", &sweep_seed);
  SeedRegistry::register_generator("raw", &raw_iso_weekday_seed);
  SeedRegistry::register_generator("geo_balanced", &geo_balanced_seed);
  SeedRegistry::register_generator("geo_regret2", &geo_regret2_seed);
}

Seed passthrough_seed(const std::vector<Task> &tasks)
{
  // Even passthrough: normalize the single kept option per task if present
  Seed s;
  s.name = "passthrough";
  s.tasks.reserve(tasks.size());
  for (const auto &t : tasks)
  {
    if (t.day_options.empty())
    {
      s.tasks.push_back(t);
      continue;
    }
    s.tasks.push_back(choose_option(t, t.day_options.front()));
  }
  return s;
}

// NEW: “raw” generator that just emits the first option per task
// but forces weekday to 1..7 (Mon=1..Sun=7) for consumers/validators
// that expect ISO weekdays.
Seed raw_iso_weekday_seed(const std::vector<Task> &tasks)
{
  Seed s;
  s.name = "raw";
  s.tasks.reserve(tasks.size());
  for (const auto &t : tasks)
  {
    if (t.day_options.empty())
    {
      s.tasks.push_back(t);
      continue;
    }
    Task out = t;
    out.day_options.clear();
    Task::DayOption o = t.day_options.front();

    // Prefer authoritative date → ISO 1..7
    if (!o.date.empty())
    {
      o.weekday = weekday1_from_date(o.date); // 1..7
    }
    else
    {
      // Fallback: map internal 0..6 (Mon=0..Sun=6) → 1..7
      // Mon(0)->1, Tue(1)->2, ..., Sun(6)->7
      int w0 = o.weekday;
      if (w0 >= 0 && w0 <= 6)
        o.weekday = (w0 == 6) ? 7 : (w0 + 1);
      else
        o.weekday = 1; // safe default
    }

    out.day_options.push_back(std::move(o));
    s.tasks.push_back(std::move(out));
  }
  return s;
}

// --- two_week_rollout ------------------------------------------------------
// Canonical weekday by schema = mode of observed ISO weekdays (by first option).
// For alternating, prefer canonical; on alternate weeks try +1 (wrap) if available.
Seed two_week_rollout_seed(const std::vector<Task> &tasks)
{
  // Canonical weekday by schema (mode over ISO 1..7)
  std::unordered_map<int, std::array<int, 8>> counts;
  for (const auto &t : tasks)
  {
    if (t.day_options.empty())
      continue;
    int w = iso_weekday_from_date(t.day_options.front().date); // normalize
    if (counts[t.schema_number][0] == 0)
      counts[t.schema_number].fill(0);
    counts[t.schema_number][w]++;
  }
  std::unordered_map<int, int> canon;
  for (auto &kv : counts)
  {
    int best_w = 1, best_c = -1;
    for (int w = 1; w <= 7; ++w)
      if (kv.second[w] > best_c)
      {
        best_c = kv.second[w];
        best_w = w;
      }
    canon[kv.first] = best_w;
  }

  Seed s;
  s.name = "two_week_rollout";
  s.tasks.reserve(tasks.size());
  for (const auto &t : tasks)
  {
    if (t.day_options.empty())
    {
      s.tasks.push_back(t);
      continue;
    }

    int cw = canon.count(t.schema_number) ? canon[t.schema_number]
                                          : iso_weekday_from_date(t.day_options.front().date);
    int week = t.week;

    auto pick = [&](int want_wd) -> const Task::DayOption *
    {
      for (const auto &o : t.day_options)
      {
        if (iso_weekday_from_date(o.date) == want_wd)
          return &o;
      }
      return nullptr;
    };

    const Task::DayOption *chosen = nullptr;
    if (week % 2 == 1)
    {
      chosen = pick(cw);
      if (!chosen)
        chosen = pick((cw % 7) + 1); // neighbor +1
      if (!chosen && cw == 1)
        chosen = pick(7); // wrap neighbor for Monday case
    }
    else
    {
      chosen = pick((cw % 7) + 1);
      if (!chosen && cw == 1)
        chosen = pick(7);
      if (!chosen)
        chosen = pick(cw);
    }
    if (!chosen)
      chosen = &t.day_options.front();

    s.tasks.push_back(choose_option(t, *chosen));
  }
  return s;
}

// --- capacity_balanced -----------------------------------------------------
// Greedily pack tasks onto the least-loaded allowed date (by duration sum).
Seed capacity_balanced_seed(const std::vector<Task> &tasks)
{
  std::set<std::string> all_dates;
  for (const auto &t : tasks)
    for (const auto &o : t.day_options)
      all_dates.insert(o.date);

  std::unordered_map<std::string, double> load; // minutes per date
  for (auto &d : all_dates)
    load[d] = 0.0;

  // Place fixed first, then flexible; longer first within same flexibility
  std::vector<const Task *> order;
  order.reserve(tasks.size());
  for (const auto &t : tasks)
    order.push_back(&t);
  std::stable_sort(order.begin(), order.end(),
                   [](const Task *a, const Task *b)
                   {
                     const int na = (int)a->day_options.size();
                     const int nb = (int)b->day_options.size();
                     if (na != nb)
                       return na < nb;                 // fixed first
                     return a->duration > b->duration; // longer first
                   });

  Seed s;
  s.name = "capacity_balanced";
  s.tasks.reserve(tasks.size());
  for (const auto *tp : order)
  {
    const Task &t = *tp;
    if (t.day_options.empty())
    {
      s.tasks.push_back(t);
      continue;
    }

    const Task::DayOption *best = nullptr;
    double best_load = 1e100;

    for (const auto &o : t.day_options)
    {
      const double L = load[o.date];
      if (L < best_load)
      {
        best_load = L;
        best = &o;
      }
    }
    if (!best)
      best = &t.day_options.front();

    s.tasks.push_back(choose_option(t, *best));
    load[best->date] += t.duration;
  }
  return s;
}

// --- sweep ---------------------------------------- t-------------------------
// Assign each task a single date by lightest bucket; routing done later.
// (Weekday normalization is enforced when we materialize the chosen option.)
Seed sweep_seed(const std::vector<Task> &tasks)
{
  // Group options by date
  std::unordered_map<std::string, std::vector<const Task *>> by_date;
  for (const auto &t : tasks)
    for (const auto &o : t.day_options)
    {
      by_date[o.date].push_back(&t);
    }

  // Pseudo angle anchors (kept for determinism if you extend sweep later)
  int max_client = 0;
  for (const auto &t : tasks)
    max_client = std::max(max_client, t.client_id);
  (void)max_client; // currently unused

  Seed s;
  s.name = "sweep";
  s.tasks.reserve(tasks.size());

  // Lightest-bucket date choice per task (simple, stable)
  std::unordered_map<std::string, int> bucket_load; // date→count
  for (auto &kv : by_date)
    bucket_load[kv.first] = 0;

  for (const auto &t : tasks)
  {
    if (t.day_options.empty())
    {
      s.tasks.push_back(t);
      continue;
    }

    const Task::DayOption *best = nullptr;
    int bestLoad = INT_MAX;
    for (const auto &o : t.day_options)
    {
      int L = bucket_load[o.date];
      if (L < bestLoad)
      {
        bestLoad = L;
        best = &o;
      }
    }
    if (!best)
      best = &t.day_options.front();

    bucket_load[best->date] += 1;
    s.tasks.push_back(choose_option(t, *best));
  }
  return s;
}
