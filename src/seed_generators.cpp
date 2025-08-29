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
static const TimeMatrix* G_M = nullptr;
void seed_generators_set_matrix(const TimeMatrix* M) { G_M = M; }
static inline int D(int a, int b) { return (G_M ? G_M->at(a, b) : 0); } // minutes

// --- helpers ---------------------------------------------------------------
struct DayBucket {
  std::string date;               // YYYY-MM-DD
  int medoid_client = 0;          // current medoid client id
  std::vector<int> clients;       // client ids assigned to this date (no depot)
  int load_minutes = 0;           // sum of task durations (service)
};

static int recompute_medoid(const std::vector<int>& clients) {
  if (clients.empty()) return 0;
  int best = clients[0], best_sum = INT_MAX;
  for (int i : clients) {
    int s = 0;
    for (int j : clients) s += D(i, j);
    if (s < best_sum) { best_sum = s; best = i; }
  }
  return best;
}

// Incremental geo “cost” of adding client c to bucket b.
// Approximate Δ by distance to current medoid or depot, whichever smaller.
static inline int delta_geo_cost(const DayBucket& b, int client_id) {
  if (b.clients.empty()) return D(0, client_id) + D(client_id, 0); // depot star
  int to_med = D(client_id, b.medoid_client);
  int to_dep = D(0, client_id);
  return std::min(to_med, to_dep);
}

// Compute an average target load per active date to softly balance
static int target_load_per_date_minutes(const std::vector<Task>& tasks,
                                        const std::vector<std::string>& dates) {
  long long total = 0;
  for (auto& t : tasks) total += (long long)std::llround(t.duration);
  int nd = std::max(1, (int)dates.size());
  return (int)std::max(1LL, total / nd);
}

// Build unique, sorted list of candidate dates that appear in day_options
static std::vector<std::string> collect_dates(const std::vector<Task>& tasks) {
  std::set<std::string> s;
  for (auto& t : tasks) for (auto& o : t.day_options) if (!o.date.empty()) s.insert(o.date);
  return std::vector<std::string>(s.begin(), s.end());
}

static Task choose_option_unchanged(const Task& t, const std::string& dateToKeep) {
  Task out = t;
  out.day_options.clear();
  for (const auto& o : t.day_options) if (o.date == dateToKeep) {
    out.day_options.push_back(o);
    return out;
  }
  // fallback
  out.day_options.push_back(t.day_options.front());
  return out;
}

// --- geo_balanced: greedy geo + load penalty --------------------------------
Seed geo_balanced_seed(const std::vector<Task>& tasks) {
  Seed s; s.name = "geo_balanced"; s.tasks.reserve(tasks.size());
  if (!G_M) return passthrough_seed(tasks); // safety: matrix not set

  auto dates = collect_dates(tasks);
  std::unordered_map<std::string, DayBucket> B;
  for (auto& d : dates) { DayBucket b; b.date = d; B.emplace(d, std::move(b)); }

  const int target_load = target_load_per_date_minutes(tasks, dates);
  const double beta = 0.8; // load penalty weight

  // Place fixed first (few options), then longer ones
  std::vector<const Task*> order; order.reserve(tasks.size());
  for (auto& t : tasks) order.push_back(&t);
  std::stable_sort(order.begin(), order.end(),
    [](const Task* a, const Task* b){
      if (a->day_options.size() != b->day_options.size())
        return a->day_options.size() < b->day_options.size();
      return a->duration > b->duration;
    });

  for (const Task* tp : order) {
    const Task& t = *tp;
    if (t.day_options.empty()) { s.tasks.push_back(t); continue; }

    const Task::DayOption* best_o = nullptr;
    double best_score = 1e18;

    for (const auto& o : t.day_options) {
      auto& b = B[o.date];
      if (!b.clients.empty() && b.medoid_client == 0) b.medoid_client = recompute_medoid(b.clients);
      int dgeo = delta_geo_cost(b, t.client_id);
      double load_ratio = double(b.load_minutes) / std::max(1, target_load);
      double score = dgeo * (1.0 + beta * load_ratio);
      if (score < best_score) { best_score = score; best_o = &o; }
    }

    if (!best_o) best_o = &t.day_options.front();
    auto& bb = B[best_o->date];
    bb.clients.push_back(t.client_id);
    bb.load_minutes += (int)std::llround(t.duration);
    if (bb.clients.size() == 1 || (bb.clients.size() % 16 == 0))
      bb.medoid_client = recompute_medoid(bb.clients);

    s.tasks.push_back(choose_option_unchanged(t, best_o->date));
  }
  return s;
}

// --- geo_regret2: regret-2 across dates (geo cost + load) -------------------
Seed geo_regret2_seed(const std::vector<Task>& tasks) {
  Seed s; s.name = "geo_regret2"; s.tasks.reserve(tasks.size());
  if (!G_M) return passthrough_seed(tasks); // safety

  auto dates = collect_dates(tasks);
  std::unordered_map<std::string, DayBucket> B;
  for (auto& d : dates) { DayBucket b; b.date = d; B.emplace(d, std::move(b)); }

  const int target_load = target_load_per_date_minutes(tasks, dates);
  const double beta = 0.8;   // load penalty
  const double gamma = 0.05; // slight bias to pick hard tasks

  // Unassigned pool
  std::vector<const Task*> pool; pool.reserve(tasks.size());
  for (auto& t : tasks) pool.push_back(&t);

  // fixed first; long first within same flexibility
  std::stable_sort(pool.begin(), pool.end(),
    [](const Task* a, const Task* b){
      if (a->day_options.size() != b->day_options.size())
        return a->day_options.size() < b->day_options.size();
      return a->duration > b->duration;
    });

  while (!pool.empty()) {
    int pick_idx = 0;
    double best_regret = -1e18;

    const Task::DayOption* best_o1 = nullptr;
    double s1_best = 1e18, s2_best = 1e18;

    for (int i = 0; i < (int)pool.size(); ++i) {
      const Task& t = *pool[i];
      if (t.day_options.empty()) { pick_idx = i; best_regret = 1e18; break; }

      double s1 = 1e18, s2 = 1e18;
      const Task::DayOption *o1 = nullptr, *o2 = nullptr;

      for (const auto& o : t.day_options) {
        auto& b = B[o.date];
        if (!b.clients.empty() && b.medoid_client == 0) b.medoid_client = recompute_medoid(b.clients);
        int dgeo = delta_geo_cost(b, t.client_id);
        double load_ratio = double(b.load_minutes) / std::max(1, target_load);
        double score = dgeo * (1.0 + beta * load_ratio);

        if (score < s1) { s2 = s1; o2 = o1; s1 = score; o1 = &o; }
        else if (score < s2) { s2 = score; o2 = &o; }
      }

      double regret2 = (s2 - s1) + gamma * s1;
      if (regret2 > best_regret) {
        best_regret = regret2;
        pick_idx = i;
        best_o1 = (o1 ? o1 : &t.day_options.front());
        s1_best = s1; s2_best = s2;
      }
    }

    const Task* tp = pool[pick_idx];
    const Task::DayOption* best_o = best_o1 ? best_o1 : &tp->day_options.front();

    auto& b = B[best_o->date];
    b.clients.push_back(tp->client_id);
    b.load_minutes += (int)std::llround(tp->duration);
    if (b.clients.size() == 1 || (b.clients.size() % 16 == 0))
      b.medoid_client = recompute_medoid(b.clients);

    s.tasks.push_back(choose_option_unchanged(*tp, best_o->date));
    pool.erase(pool.begin() + pick_idx);
  }

  return s;
}
// --- ISO weekday utilities -------------------------------------------------
// Sakamoto's algorithm: 0=Sunday..6=Saturday
static int dow_sakamoto(int y, int m, int d) {
  static const int t[12] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  if (m < 3) y -= 1;
  return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
}

// Parse "YYYY-MM-DD" → ISO weekday 1..7 (Mon=1..Sun=7)
static int iso_weekday_from_date(const std::string& ymd) {
  // Minimal, fast parser (assumes valid YYYY-MM-DD)
  int y = 0, m = 0, d = 0;
  // Safer than std::stoi substr in tight loops
  // Format: 0123456789
  //         YYYY-MM-DD
  if (ymd.size() >= 10) {
    y = (ymd[0]-'0')*1000 + (ymd[1]-'0')*100 + (ymd[2]-'0')*10 + (ymd[3]-'0');
    m = (ymd[5]-'0')*10   + (ymd[6]-'0');
    d = (ymd[8]-'0')*10   + (ymd[9]-'0');
  }
  int dow = dow_sakamoto(y, m, d); // 0=Sun..6=Sat
  return (dow == 0) ? 7 : dow;     // ISO 1..7
}

// Normalize a DayOption's weekday to ISO 1..7 using its date
static Task::DayOption normalized_option(const Task::DayOption& in) {
  Task::DayOption out = in;
  out.weekday = iso_weekday_from_date(in.date);
  return out;
}

// Helper: choose option and force ISO weekday
static Task choose_option(const Task& t, const Task::DayOption& opt) {
  Task out = t;
  out.day_options.clear();
  out.day_options.push_back(normalized_option(opt));
  return out;
}

// (Kept but not used externally; safe)
static int wd_of(const Task& t) {
  return t.day_options.empty()
           ? 0
           : iso_weekday_from_date(t.day_options.front().date);
}
static const std::string& date_of(const Task& t) { return t.day_options.front().date; }

void register_default_seed_generators() {
  SeedRegistry::register_generator("passthrough", &passthrough_seed);
  SeedRegistry::register_generator("two_week_rollout", &two_week_rollout_seed);
  SeedRegistry::register_generator("capacity_balanced", &capacity_balanced_seed);
  SeedRegistry::register_generator("sweep", &sweep_seed);
  SeedRegistry::register_generator("raw", &raw_iso_weekday_seed);
  SeedRegistry::register_generator("geo_balanced", &geo_balanced_seed);
  SeedRegistry::register_generator("geo_regret2", &geo_regret2_seed);
}

Seed passthrough_seed(const std::vector<Task>& tasks) {
  // Even passthrough: normalize the single kept option per task if present
  Seed s; s.name = "passthrough"; s.tasks.reserve(tasks.size());
  for (const auto& t : tasks) {
    if (t.day_options.empty()) { s.tasks.push_back(t); continue; }
    s.tasks.push_back(choose_option(t, t.day_options.front()));
  }
  return s;
}

// NEW: “raw” generator that just emits the first option per task
// but forces weekday to 1..7 (Mon=1..Sun=7) for consumers/validators
// that expect ISO weekdays.
Seed raw_iso_weekday_seed(const std::vector<Task>& tasks) {
  Seed s; s.name = "raw"; s.tasks.reserve(tasks.size());
  for (const auto& t : tasks) {
    if (t.day_options.empty()) { s.tasks.push_back(t); continue; }
    Task out = t;
    out.day_options.clear();
    Task::DayOption o = t.day_options.front();

    // Prefer authoritative date → ISO 1..7
    if (!o.date.empty()) {
      o.weekday = weekday1_from_date(o.date);         // 1..7
    } else {
      // Fallback: map internal 0..6 (Mon=0..Sun=6) → 1..7
      // Mon(0)->1, Tue(1)->2, ..., Sun(6)->7
      int w0 = o.weekday;
      if (w0 >= 0 && w0 <= 6) o.weekday = (w0 == 6) ? 7 : (w0 + 1);
      else o.weekday = 1; // safe default
    }

    out.day_options.push_back(std::move(o));
    s.tasks.push_back(std::move(out));
  }
  return s;
}

// --- two_week_rollout ------------------------------------------------------
// Canonical weekday by schema = mode of observed ISO weekdays (by first option).
// For alternating, prefer canonical; on alternate weeks try +1 (wrap) if available.
Seed two_week_rollout_seed(const std::vector<Task>& tasks) {
  // Canonical weekday by schema (mode over ISO 1..7)
  std::unordered_map<int, std::array<int,8>> counts;
  for (const auto& t : tasks) {
    if (t.day_options.empty()) continue;
    int w = iso_weekday_from_date(t.day_options.front().date); // normalize
    if (counts[t.schema_number][0] == 0) counts[t.schema_number].fill(0);
    counts[t.schema_number][w]++;
  }
  std::unordered_map<int,int> canon;
  for (auto& kv : counts) {
    int best_w=1, best_c=-1;
    for (int w=1; w<=7; ++w) if (kv.second[w] > best_c) { best_c = kv.second[w]; best_w = w; }
    canon[kv.first] = best_w;
  }

  Seed s; s.name = "two_week_rollout"; s.tasks.reserve(tasks.size());
  for (const auto& t : tasks) {
    if (t.day_options.empty()) { s.tasks.push_back(t); continue; }

    int cw = canon.count(t.schema_number) ? canon[t.schema_number]
                                          : iso_weekday_from_date(t.day_options.front().date);
    int week = t.week;

    auto pick = [&](int want_wd) -> const Task::DayOption* {
      for (const auto& o : t.day_options) {
        if (iso_weekday_from_date(o.date) == want_wd) return &o;
      }
      return nullptr;
    };

    const Task::DayOption* chosen = nullptr;
    if (week % 2 == 1) {
      chosen = pick(cw);
      if (!chosen) chosen = pick((cw % 7) + 1); // neighbor +1
      if (!chosen && cw == 1) chosen = pick(7); // wrap neighbor for Monday case
    } else {
      chosen = pick((cw % 7) + 1);
      if (!chosen && cw == 1) chosen = pick(7);
      if (!chosen) chosen = pick(cw);
    }
    if (!chosen) chosen = &t.day_options.front();

    s.tasks.push_back(choose_option(t, *chosen));
  }
  return s;
}

// --- capacity_balanced -----------------------------------------------------
// Greedily pack tasks onto the least-loaded allowed date (by duration sum).
Seed capacity_balanced_seed(const std::vector<Task>& tasks) {
  std::set<std::string> all_dates;
  for (const auto& t: tasks) for (const auto& o: t.day_options) all_dates.insert(o.date);

  std::unordered_map<std::string,double> load; // minutes per date
  for (auto& d : all_dates) load[d] = 0.0;

  // Place fixed first, then flexible; longer first within same flexibility
  std::vector<const Task*> order; order.reserve(tasks.size());
  for (const auto& t : tasks) order.push_back(&t);
  std::stable_sort(order.begin(), order.end(),
    [](const Task* a, const Task* b){
      const int na = (int)a->day_options.size();
      const int nb = (int)b->day_options.size();
      if (na != nb) return na < nb;             // fixed first
      return a->duration > b->duration;         // longer first
    });

  Seed s; s.name = "capacity_balanced"; s.tasks.reserve(tasks.size());
  for (const auto* tp : order) {
    const Task& t = *tp;
    if (t.day_options.empty()) { s.tasks.push_back(t); continue; }

    const Task::DayOption* best = nullptr;
    double best_load = 1e100;

    for (const auto& o : t.day_options) {
      const double L = load[o.date];
      if (L < best_load) { best_load = L; best = &o; }
    }
    if (!best) best = &t.day_options.front();

    s.tasks.push_back(choose_option(t, *best));
    load[best->date] += t.duration;
  }
  return s;
}

// --- sweep ---------------------------------------- t-------------------------
// Assign each task a single date by lightest bucket; routing done later.
// (Weekday normalization is enforced when we materialize the chosen option.)
Seed sweep_seed(const std::vector<Task>& tasks) {
  // Group options by date
  std::unordered_map<std::string, std::vector<const Task*>> by_date;
  for (const auto& t : tasks) for (const auto& o : t.day_options) {
    by_date[o.date].push_back(&t);
  }

  // Pseudo angle anchors (kept for determinism if you extend sweep later)
  int max_client = 0;
  for (const auto& t: tasks) max_client = std::max(max_client, t.client_id);
  (void)max_client; // currently unused

  Seed s; s.name = "sweep"; s.tasks.reserve(tasks.size());

  // Lightest-bucket date choice per task (simple, stable)
  std::unordered_map<std::string, int> bucket_load; // date→count
  for (auto& kv : by_date) bucket_load[kv.first] = 0;

  for (const auto& t : tasks) {
    if (t.day_options.empty()) { s.tasks.push_back(t); continue; }

    const Task::DayOption* best = nullptr; int bestLoad = INT_MAX;
    for (const auto& o : t.day_options) {
      int L = bucket_load[o.date];
      if (L < bestLoad) { bestLoad = L; best = &o; }
    }
    if (!best) best = &t.day_options.front();

    bucket_load[best->date] += 1;
    s.tasks.push_back(choose_option(t, *best));
  }
  return s;
}
