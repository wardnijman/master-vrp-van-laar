// cache.h
#pragma once
#include "types.h"
#include <string>
#include <unordered_map>
#include <list>

struct DayCacheKey {
  int day_index;
  uint64_t hash_ids;
  bool operator==(const DayCacheKey& o) const {
    return day_index == o.day_index && hash_ids == o.hash_ids;
  }
};

// What we store per day in the cache.
struct DayCacheValue {
  DayResult result;
  // Warm-startable routes in GLOBAL client IDs (no depot entries).
  std::vector<std::vector<int>> routes_global;   // <-- add this
};

struct DayCache {
  size_t capacity = 512;

  std::list<DayCacheKey> order;
  using ListIt = std::list<DayCacheKey>::iterator;

  std::unordered_map<std::string, std::pair<ListIt, DayCacheValue>> map;

  static std::string kstr(const DayCacheKey& k);
  static uint64_t hash_task_ids(const std::vector<TaskRef>& tasks);

  bool get(const DayCacheKey& k, DayCacheValue& out);
  void put(const DayCacheKey& k, const DayCacheValue& v);
};
