#pragma once
#include "seed.h"

// Call once at startup to register built-ins.
void register_default_seed_generators();

// A trivial “passthrough” generator: assumes tasks already carry exactly one day_option.
// Useful when you feed a handcrafted seed JSON to the program.
Seed passthrough_seed(const std::vector<Task>& tasks);

// Example stub for future generators (S1/S3/S4/…): keep the interface identical.
Seed two_week_rollout_seed(const std::vector<Task>& tasks);
Seed capacity_balanced_seed(const std::vector<Task>& tasks);
Seed sweep_seed(const std::vector<Task>& tasks);
