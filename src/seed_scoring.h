#pragma once
#include "seed.h"
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>

struct RankedSeed {
  Seed seed;
  SeedScore score;
};

// Evaluate a list of generator names on the same base task list; keep top-k.
std::vector<RankedSeed> build_and_rank_seeds(
  const std::vector<std::string>& generator_names,
  const std::vector<Task>& base_tasks,
  const EvalConfig& cfg,
  int top_k);

// Utility: stable sort by score (ascending).
void sort_by_cost(std::vector<RankedSeed>& v);


// Small helper to pretty print a ranked list
static void print_ranked_seeds(const std::vector<RankedSeed>& ranked) {
  std::cout << "\n# Seed ranking\n";
  std::cout << std::left << std::setw(24) << "name"
            << std::right << std::setw(16) << "cost"
            << std::setw(10) << "valid"
            << "   " << "note" << "\n";
  for (const auto& r : ranked) {
    std::cout << std::left << std::setw(24) << r.seed.name
              << std::right << std::setw(16) << std::fixed << std::setprecision(2) << r.score.total_cost
              << std::setw(10) << (r.score.valid ? "yes" : "no")
              << "   " << (r.score.validation_error.empty() ? "" : r.score.validation_error)
              << "\n";
  }
  std::cout << std::endl;
}