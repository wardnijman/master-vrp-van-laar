#include "seed_generators.h"
#include <algorithm>

void register_default_seed_generators() {
  SeedRegistry::register_generator("passthrough", &passthrough_seed);
  SeedRegistry::register_generator("two_week_rollout", &two_week_rollout_seed);
  SeedRegistry::register_generator("capacity_balanced", &capacity_balanced_seed);
  SeedRegistry::register_generator("sweep", &sweep_seed);
}

Seed passthrough_seed(const std::vector<Task>& tasks) {
  // Validate “shape”: exactly one day_option per task (seed shape), but not full validation yet.
  bool ok = std::all_of(tasks.begin(), tasks.end(), [](const Task& t){ return t.day_options.size() == 1; });
  Seed s;
  s.name = "passthrough";
  s.tasks = tasks; // copy regardless; full validation happens later
  return s;
}

// Stubs – implement later. Returning passthrough for now keeps compile OK.
Seed two_week_rollout_seed(const std::vector<Task>& tasks) {
  Seed s; s.name = "two_week_rollout"; s.tasks = tasks; return s;
}
Seed capacity_balanced_seed(const std::vector<Task>& tasks) {
  Seed s; s.name = "capacity_balanced"; s.tasks = tasks; return s;
}
Seed sweep_seed(const std::vector<Task>& tasks) {
  Seed s; s.name = "sweep"; s.tasks = tasks; return s;
}
