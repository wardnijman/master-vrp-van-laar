#include "cache.h"
#include <algorithm>

std::string DayCache::kstr(const DayCacheKey& k) {
  return std::to_string(k.day_index) + ":" + std::to_string(k.hash_ids);
}

uint64_t DayCache::hash_task_ids(const std::vector<TaskRef>& tasks) {
  std::vector<std::string> ids; ids.reserve(tasks.size());
  for (const auto& t : tasks) ids.push_back(t.task_id);
  std::sort(ids.begin(), ids.end());

  // FNV-1a 64-bit
  uint64_t h = 1469598103934665603ull;
  for (const auto& s : ids) {
    for (unsigned char c : s) {
      h ^= c;
      h *= 1099511628211ull;
    }
  }
  return h;
}

bool DayCache::get(const DayCacheKey& k, DayCacheValue& out) {
  const std::string key = kstr(k);
  auto it = map.find(key);
  if (it == map.end()) return false;

  // Move the node to the front (MRU)
  order.splice(order.begin(), order, it->second.first);

  out = it->second.second; // includes result + routes_global
  return true;
}

void DayCache::put(const DayCacheKey& k, const DayCacheValue& v) {
  const std::string key = kstr(k);
  auto it = map.find(key);

  if (it != map.end()) {
    // Overwrite value and promote to MRU
    it->second.second = v; // includes result + routes_global
    order.splice(order.begin(), order, it->second.first);
    return;
  }

  // Evict LRU if full
  if (map.size() >= capacity && !order.empty()) {
    const DayCacheKey& lru_key = order.back();
    map.erase(kstr(lru_key));
    order.pop_back();
  }

  // Insert new as MRU
  order.push_front(k);
  map.emplace(key, std::make_pair(order.begin(), v));
}
