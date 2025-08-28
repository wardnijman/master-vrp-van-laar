// types.h
#pragma once
#include <vector>
#include <string>
#include <cstdint>
#include <optional>

struct TaskRef {
  std::string task_id;     // original ID
  int client_idx;          // index into matrix (0=depot)
  int schema_number;       // e.g. 814
  char suffix;             // 'a','b',... or '\0' if none
  int week;                // 1..12 (or 1..N)
  std::vector<int> weekday_options; // allowed weekdays [0..6], 0=Mon
  int service_minutes;     // service time at node
};

struct DayRoute {
  std::vector<int> sequence_client_indices; // includes depot at start/end if you like
  int route_minutes = 0;    // travel + service
};

struct DayResult {
  bool feasible = true;
  int total_minutes = 0;         // sum of all routes (travel+service)
  int used_vehicles = 0;
  int overtime_minutes = 0;      // aggregated over routes
  std::vector<DayRoute> routes;
};

struct DayWarmStart {
  // Optional seed sequences per vehicle (client indices without depot is OK)
  std::vector<std::vector<int>> vehicle_sequences; 
  std::vector<std::vector<int>> routes_global;

};

// compressed, read-only view of matrix in minutes (time). 0 is depot.
struct TimeMatrix {
  int n = 0;
  // access with m[src * n + dst]
  std::vector<int> m; // minutes
  inline int at(int i, int j) const { return m[i * n + j]; }
};

struct DayConfig {
  int vehicles = 25;
  int route_limit_minutes = 9 * 60; // hard horizon
  int soft_upper_cost = 7;          // per minute above horizon
  int span_coeff = 1;               // SetSpanCostCoefficientForAllVehicles
  int exploration_seconds = 2;      // short solve
  int intensify_seconds = 60;       // long solve
};
