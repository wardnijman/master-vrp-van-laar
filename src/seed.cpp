#include "seed.h"
#include <stdexcept>

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
