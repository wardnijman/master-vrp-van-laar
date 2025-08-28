#include "validation.h"
#include "third_party/json/single_include/nlohmann/json.hpp"

#include <unordered_set>
#include <regex>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <set>
#include <unordered_set>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
using json = nlohmann::json;

namespace vl
{

    inline bool is_blank(const std::string &s)
    {
        return std::all_of(s.begin(), s.end(), [](unsigned char c)
                           { return std::isspace(c); });
    }

    [[noreturn]] void fail(const std::string &msg)
    {
        throw std::runtime_error(msg);
    }
    static std::vector<Task> parse_tasks(const nlohmann::json &arr)
    {
        std::vector<Task> out;
        out.reserve(arr.size());
        for (const auto &j : arr)
        {
            Task t;
            t.task_id = j.at("task_id").get<std::string>();
            t.schema_number = j.at("schema_number").get<int>();
            t.client_id = j.at("client_id").get<int>();
            t.duration = j.at("duration").get<double>();
            t.week = j.at("week").get<int>();
            if (j.contains("interval"))
                t.interval = j.at("interval").get<int>();
            if (j.contains("is_intraweek"))
                t.is_intraweek = j.at("is_intraweek").get<bool>();
            if (j.contains("inferred_weekdays"))
                t.inferred_weekdays = j.at("inferred_weekdays").get<bool>();

            if (j.contains("frequency"))
            {
                t.frequency.value = j["frequency"].value("value", 1.0);
                t.frequency.unit = j["frequency"].value("unit", std::string{"week"});
            }
            if (j.contains("address"))
            {
                t.address.street = j["address"].value("street", "");
                t.address.city = j["address"].value("city", "");
                t.address.province = j["address"].value("province", "");
            }
            if (j.contains("location"))
            {
                t.location.lat = j["location"].value("lat", 0.0);
                t.location.lng = j["location"].value("lng", 0.0);
            }
            if (j.contains("day_options"))
            {
                for (const auto &d : j["day_options"])
                {
                    DayOption o;
                    o.date = d.value("date", "");
                    o.weekday = d.value("weekday", -1);
                    t.day_options.push_back(o);
                    if (o.weekday >= 0)
                        t.weekday_options.push_back(o.weekday);
                }
            }

            out.push_back(std::move(t));
        }
        return out;
    }

    static Matrix parse_matrix(const nlohmann::json &m)
    {
        Matrix out;
        out.reserve(m.size());
        for (const auto &row : m)
        {
            std::vector<double> r;
            r.reserve(row.size());
            for (const auto &v : row)
                r.push_back(v.get<double>());
            out.push_back(std::move(r));
        }
        return out;
    }

    static Config parse_config(const nlohmann::json &c)
    {
        Config cfg;
        cfg.max_hours_per_route = c.value("MAX_HOURS_PER_DAY", 9.0);
        cfg.max_routes_per_day = c.value("VEHICLES_PER_DAY", 25);
        cfg.time_limit_seconds = c.value("TIME_LIMIT_SECONDS", 30);
        cfg.use_matrix_cache = c.value("USE_MATRIX_CACHE", false);
        return cfg;
    }

    // New entrypoint from json
    void validate_all(const nlohmann::json &tasks_json,
                      const nlohmann::json &matrix_json,
                      const nlohmann::json &config_json)
    {
        auto tasks = parse_tasks(tasks_json);
        auto matrix = parse_matrix(matrix_json);
        auto config = parse_config(config_json);

        // now call the "real" validator
        validate_all(tasks, matrix, config);
    }

    // validation.cpp — paste inside namespace vl
    void validate_all(const std::vector<Task> &tasks,
                      const Matrix &matrix,
                      const Config &config)
    {
        // 1) Base field checks
        validate_tasks(tasks, matrix.size());

        // 2) Matrix sanity + alignment
        validate_matrix_task_alignment(tasks, matrix);
        validate_matrix_distance_range(matrix);
        validate_unreachable_clients(matrix);

        // 3) Per-task caps
        validate_single_task_duration(tasks, config);

        // 4) Weekly capacity
        validate_weekly_capacity_projection(tasks, config);

        // 5) Planning horizon derived from max week
        int max_week = 0;
        for (const auto &t : tasks)
            max_week = std::max(max_week, t.week);
        const int planning_days = max_week * 7;
        if (planning_days > 0)
        {
            validate_day_options_within_planning(tasks, planning_days);
            validate_assignment_possible(tasks, planning_days);
            validate_dayoption_overload(tasks, config, planning_days);
        }

        // 6) Workload across horizon (service + travel + overhead)
        std::vector<TaskPlan> plans;
        plans.reserve(tasks.size());
        for (const auto &t : tasks)
            plans.push_back(TaskPlan{t.task_id, t.week, t.duration, t.client_id});
        validate_total_workload(plans, matrix, config);

        // 7) Schema / recurrence integrity
        validate_interval_positive(tasks);
        validate_week_number_range(tasks);
        validate_schema_week_gaps(tasks);
        validate_schema_duplicates(tasks);
        validate_dayoption_alignment(tasks);
        validate_dayoption_weekday_bounds(tasks);
        validate_schema_canonical_structure(tasks);

        // 8) Soft warning on synthetic-looking matrix
        validate_no_placeholder_matrix(matrix);

        // NOTE: validate_task_spacing_within_schema(...) is intentionally not called here
        // (treated as output/assignment-time validation rather than input-shape validation).
    }

    void validate_tasks(const std::vector<Task> &tasks, std::size_t matrix_size)
    {
        if (matrix_size == 0)
            fail("Distance matrix is empty.");
        const int max_index = static_cast<int>(matrix_size) - 1;

        std::unordered_set<std::string> seen_ids;
        seen_ids.reserve(tasks.size() * 2);

        for (std::size_t i = 0; i < tasks.size(); ++i)
        {
            const Task &t = tasks[i];

            // ---- Required-like checks (acting as REQUIRED_FIELDS) ----
            if (t.task_id.empty())
                fail("Task " + std::to_string(i) + " missing required field: task_id");
            if (t.schema_number <= 0)
                fail("Task " + t.task_id + " has invalid schema_number (<=0).");
            if (t.duration <= 0.0 || !std::isfinite(t.duration))
                fail("Task " + t.task_id + " has invalid duration.");
            if (t.weekday_options.empty())
                fail("Task " + t.task_id + " has no weekday_options.");

            // frequency
            if (t.frequency.value <= 0)
                fail("Task " + t.task_id + " has invalid frequency.value.");
            if (t.frequency.unit.empty() || is_blank(t.frequency.unit))
                fail("Task " + t.task_id + " has invalid frequency.unit.");

            // address
            if (t.address.street.empty() || is_blank(t.address.street))
                fail("Task " + t.task_id + " has incomplete address: missing or invalid street");
            if (t.address.city.empty() || is_blank(t.address.city))
                fail("Task " + t.task_id + " has incomplete address: missing or invalid city");
            if (t.address.province.empty() || is_blank(t.address.province))
                fail("Task " + t.task_id + " has incomplete address: missing or invalid province");
            // postal_code intentionally skipped to match the commented Python lines

            // location
            if (!std::isfinite(t.location.lat))
                fail("Task " + t.task_id + " has invalid lat/lng (lat not finite).");
            if (!std::isfinite(t.location.lng))
                fail("Task " + t.task_id + " has invalid lat/lng (lng not finite).");

            // client index bounds
            if (t.client_id < 0 || t.client_id > max_index)
                fail("Task " + t.task_id + " has invalid client_id: " + std::to_string(t.client_id));

            // duplicate task_id
            if (!seen_ids.insert(t.task_id).second)
                fail("Duplicate task_id found: " + t.task_id);

            // task_id pattern:  "{schema_number}-W\\d+(-[a-z])?"
            const std::string pat = "^" + std::to_string(t.schema_number) + "-W\\d+(?:-[a-z])?$";
            static thread_local std::regex re; // reuse object storage
            re = std::regex(pat);
            if (!std::regex_match(t.task_id, re))
            {
                std::ostringstream oss;
                oss << "Task ID format invalid: " << t.task_id
                    << " (expected format: " << t.schema_number << "-W<weeknum>[-a])";
                fail(oss.str());
            }

            // Extra: mimic Python's fallback numeric check for lat stringiness (not really needed in C++)
            if (!std::isfinite(t.location.lat))
            {
                fail("Task " + t.task_id + " has non-numeric lat.");
            }
        }
    }
    static inline bool is_square(const Matrix &m)
    {
        const std::size_t n = m.size();
        if (n == 0)
            return false;
        for (const auto &row : m)
            if (row.size() != n)
                return false;
        return true;
    }

    static inline bool is_nan(double v) { return std::isnan(v); }
    static inline bool is_finite(double v) { return std::isfinite(v); }

    void validate_total_workload(const std::vector<TaskPlan> &tasks,
                                 const Matrix &matrix,
                                 const Config &config)
    {
        if (!is_square(matrix))
            fail("Distance matrix must be non-empty and square.");
        if (matrix.empty())
            fail("Distance matrix is empty.");
        if (matrix.size() == 1)
            fail("Matrix has only depot; no clients.");

        if (config.max_hours_per_route <= 0.0)
            fail("config.max_hours_per_route must be > 0.");
        if (config.max_routes_per_day <= 0)
            fail("config.max_routes_per_day must be > 0.");

        if (tasks.empty())
            return; // nothing to validate

        // planning_days = max(week) * 7  (weeks start at 1)
        int max_week = 0;
        double total_service_minutes = 0.0;

        const int max_index = static_cast<int>(matrix.size()) - 1;

        for (const auto &t : tasks)
        {
            if (t.week <= 0)
                fail("Task " + t.task_id + " has invalid week (<=0).");
            if (!is_finite(t.duration) || t.duration <= 0.0)
                fail("Task " + t.task_id + " has invalid duration.");
            if (t.client_id < 0 || t.client_id > max_index)
                fail("Task " + t.task_id + " has invalid client_id: " + std::to_string(t.client_id));

            max_week = std::max(max_week, t.week);
            total_service_minutes += t.duration;
        }

        const int planning_days = max_week * 7;
        const double route_capacity_minutes = config.max_hours_per_route * 60.0;
        const double total_route_capacity_minutes =
            static_cast<double>(config.max_routes_per_day) *
            static_cast<double>(planning_days) *
            route_capacity_minutes;

        // avg depot→client distance over all tasks (meters)
        double sum_depot_dist = 0.0;
        for (const auto &t : tasks)
        {
            double d = matrix[0][t.client_id];
            if (!is_finite(d))
                fail("Matrix has non-finite depot distance to client " + std::to_string(t.client_id));
            sum_depot_dist += d;
        }
        const double avg_depot_distance_m = sum_depot_dist / static_cast<double>(tasks.size());

        // Minimum number of routes needed from service time alone.
        const double min_routes_needed =
            std::ceil(total_service_minutes / route_capacity_minutes);

        // Travel time lower bound: depot→client→depot per route, 60 km/h ≈ 1000 m/min.
        const double min_depot_travel_per_route_minutes =
            2.0 * (avg_depot_distance_m / 1000.0);

        const double total_min_travel_minutes =
            min_routes_needed * min_depot_travel_per_route_minutes;

        // Fixed per-route overhead (loading/briefing/breaks) in minutes.
        static constexpr double per_route_overhead_minutes = 10.0;
        const double total_overhead_minutes =
            min_routes_needed * per_route_overhead_minutes;

        const double total_required_minutes =
            total_service_minutes + total_min_travel_minutes + total_overhead_minutes;

        if (total_required_minutes > total_route_capacity_minutes)
        {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1)
                << "Infeasible workload: requires at least " << total_required_minutes << " min "
                << "(service=" << total_service_minutes
                << ", travel≈" << total_min_travel_minutes
                << ", overhead=" << total_overhead_minutes << ") "
                << "but only " << std::setprecision(0) << total_route_capacity_minutes << " min available "
                << "(" << planning_days << " days × " << config.max_routes_per_day
                << " routes × " << std::setprecision(2) << config.max_hours_per_route << "h).";
            fail(oss.str());
        }
    }

    void validate_unreachable_clients(const Matrix &matrix, double z_threshold)
    {
        if (!is_square(matrix))
            fail("Distance matrix must be non-empty and square.");
        const std::size_t n = matrix.size();
        if (n == 0)
            fail("Distance matrix is empty.");

        // Reject any NaN values anywhere.
        for (std::size_t i = 0; i < n; ++i)
        {
            for (std::size_t j = 0; j < n; ++j)
            {
                if (is_nan(matrix[i][j]))
                {
                    fail("Distance matrix contains NaN values.");
                }
            }
        }

        // Convert to km, ignore diagonal; check each row has at least one finite non-diagonal value.
        std::vector<double> row_means(n, std::numeric_limits<double>::quiet_NaN());

        for (std::size_t i = 0; i < n; ++i)
        {
            std::size_t count = 0;
            double sum_km = 0.0;

            for (std::size_t j = 0; j < n; ++j)
            {
                if (i == j)
                    continue; // ignore self
                double v = matrix[i][j];
                if (is_finite(v))
                {
                    sum_km += v / 1000.0;
                    ++count;
                }
            }

            if (count == 0)
            {
                std::ostringstream oss;
                oss << "Client " << i << " has no reachable neighbors (row).";
                fail(oss.str());
            }

            row_means[i] = sum_km / static_cast<double>(count);
        }

        // Global mean and stddev over row means.
        double mean = 0.0;
        for (double v : row_means)
            mean += v;
        mean /= static_cast<double>(n);

        double var = 0.0;
        for (double v : row_means)
        {
            const double d = v - mean;
            var += d * d;
        }
        var /= static_cast<double>(n);
        const double std = std::sqrt(var);

        if (std <= 0.0)
            return; // no spread → no outliers

        // Warn on outliers
        for (std::size_t i = 0; i < n; ++i)
        {
            const double avg = row_means[i];
            if (avg > mean + z_threshold * std)
            {
                std::cerr << "⚠️ Client " << i << " is an outlier (mean distance "
                          << std::fixed << std::setprecision(1) << avg << " km > "
                          << mean << " + " << z_threshold << "×" << std << ")\n";
            }
        }
    }

    void validate_single_task_duration(const std::vector<Task> &tasks,
                                       const Config &config)
    {
        double max_minutes = config.max_hours_per_route * 60.0;
        if (max_minutes <= 0)
            fail("config.max_hours_per_route must be > 0.");
        for (const auto &t : tasks)
            if (t.duration > max_minutes)
                fail("Task " + t.task_id + " duration " + std::to_string(t.duration) +
                     " min exceeds " + std::to_string(max_minutes) + " min.");
    }

    void validate_day_options_within_planning(const std::vector<Task> &tasks,
                                              int total_days)
    {
        if (total_days <= 0)
            fail("total_days must be > 0.");
        const int max_day = total_days - 1;
        for (const auto &t : tasks)
        {
            const int base = (t.week - 1) * 7;
            bool ok = false;
            for (int wd : t.weekday_options)
            {
                if (wd < 0 || wd > 6)
                    continue;
                const int abs_day = base + wd;
                if (abs_day >= 0 && abs_day <= max_day)
                {
                    ok = true;
                    break;
                }
            }
            if (!ok)
                fail("Task " + t.task_id + " has no weekday_options within 0–" + std::to_string(max_day));
        }
    }

    void validate_weekly_capacity_projection(const std::vector<Task> &tasks,
                                             const Config &config)
    {
        if (config.max_hours_per_route <= 0.0)
            fail("config.max_hours_per_route must be > 0.");
        if (config.max_routes_per_day <= 0)
            fail("config.max_routes_per_day must be > 0.");

        std::map<int, double> weekly_totals;
        for (const auto &t : tasks)
        {
            if (t.week < 1)
                fail("Task " + t.task_id + " has invalid week (<1).");
            if (!std::isfinite(t.duration) || t.duration <= 0.0)
                fail("Task " + t.task_id + " has invalid duration.");
            weekly_totals[t.week] += t.duration;
        }

        const double max_minutes_per_week =
            config.max_hours_per_route * config.max_routes_per_day * 7.0 * 60.0;

        for (const auto &kv : weekly_totals)
        {
            const int week = kv.first;
            const double total = kv.second;
            if (total > max_minutes_per_week)
            {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(1)
                    << "Week " << week << " requires " << total << " minutes of service "
                    << "but only " << std::setprecision(0) << max_minutes_per_week << " minutes available "
                    << "(max_routes_per_day=" << config.max_routes_per_day
                    << ", max_hours_per_route=" << std::setprecision(2) << config.max_hours_per_route << ")";
                fail(oss.str());
            }
        }
    }

    void validate_schema_week_gaps(const std::vector<Task> &tasks)
    {
        std::map<int, std::vector<const Task *>> schemas;
        for (const auto &t : tasks)
            schemas[t.schema_number].push_back(&t);

        for (auto &[schema, group] : schemas)
        {
            if (group.empty())
                continue;
            double freq = group[0]->frequency.value;
            if (freq <= 0.0)
                continue;
            if (freq <= 1.0)
            {
                int interval = static_cast<int>(std::lround(1.0 / freq));
                std::set<int> uniq;
                for (auto *tp : group)
                    uniq.insert(tp->week);
                if (uniq.size() < 2)
                    continue;
                std::vector<int> weeks(uniq.begin(), uniq.end());
                std::vector<int> deltas;
                for (size_t i = 1; i < weeks.size(); ++i)
                    deltas.push_back(weeks[i] - weeks[i - 1]);
                if (!std::all_of(deltas.begin(), deltas.end(), [&](int d)
                                 { return d == interval; }))
                {
                    std::ostringstream oss;
                    oss << "Schema " << schema << " has inconsistent week gaps: [";
                    for (size_t i = 0; i < deltas.size(); ++i)
                    {
                        if (i)
                            oss << ", ";
                        oss << deltas[i];
                    }
                    oss << "] (expected " << interval << " from frequency=" << freq << ")";
                    fail(oss.str());
                }
            }
        }
    }

    void validate_schema_duplicates(const std::vector<Task> &tasks)
    {
        std::map<int, std::map<int, std::set<std::string>>> S; // schema->week->variants
        std::regex re("-W\\d+(?:-([a-z]))?$");

        for (const auto &t : tasks)
        {
            std::smatch m;
            std::string var;
            if (std::regex_search(t.task_id, m, re) && m.size() >= 2)
                var = m[1].str();
            auto &setv = S[t.schema_number][t.week];
            if (!setv.insert(var).second)
                fail("Duplicate variant '" + var + "' for schema " + std::to_string(t.schema_number) +
                     " in week " + std::to_string(t.week));
        }

        for (auto &[schema, weeks] : S)
        {
            if (weeks.empty())
                continue;
            const auto &base = weeks.begin()->second;
            for (auto &[w, vs] : weeks)
            {
                if (vs != base)
                {
                    std::ostringstream oss;
                    auto print_set = [](const std::set<std::string> &s)
                    {
                        std::ostringstream o;
                        o << "[";
                        bool first = true;
                        for (auto &e : s)
                        {
                            if (!first)
                                o << ", ";
                            first = false;
                            o << (e.empty() ? "None" : e);
                        }
                        o << "]";
                        return o.str();
                    };
                    oss << "Schema " << schema << " has inconsistent intra-week variants. "
                        << "Expected " << print_set(base) << ", but got " << print_set(vs)
                        << " in week " << w << ".";
                    fail(oss.str());
                }
            }
        }
    }

    void validate_dayoption_alignment(const std::vector<Task> &tasks)
    {
        std::map<int, std::map<std::string, std::vector<std::set<int>>>> M; // schema->variant->list of sets
        std::regex re("-W\\d+(?:-([a-z]))?$");

        for (const auto &t : tasks)
        {
            std::smatch m;
            std::string var;
            if (std::regex_search(t.task_id, m, re) && m.size() >= 2)
                var = m[1].str();
            M[t.schema_number][var].push_back(std::set<int>(t.weekday_options.begin(), t.weekday_options.end()));
        }

        for (auto &[schema, vmap] : M)
        {
            for (auto &[var, lists] : vmap)
            {
                if (lists.empty())
                    continue;
                const auto &first = lists.front();
                for (size_t i = 1; i < lists.size(); ++i)
                {
                    if (lists[i] != first)
                    {
                        std::cerr << "⚠️ Warning: schema " << schema << " variant '"
                                  << (var.empty() ? "None" : var) << "' has drifting day_options across weeks.\n";
                        break;
                    }
                }
            }
        }
    }

    void validate_assignment_possible(const std::vector<Task> &tasks, int planning_days)
    {
        for (const auto &t : tasks)
        {
            bool invalid = true;
            for (int d : t.weekday_options)
            {
                if (d >= 0 && d < planning_days)
                {
                    invalid = false;
                    break;
                }
            }
            if (invalid)
                fail("Task " + t.task_id + " has no valid day_option in planning range (0 to " +
                     std::to_string(planning_days - 1) + ")");
        }
    }

    void validate_dayoption_weekday_bounds(const std::vector<Task> &tasks)
    {
        for (const auto &t : tasks)
            for (int d : t.weekday_options)
                if (d < 0 || d > 6)
                    fail("Task " + t.task_id + " has invalid day_option: " + std::to_string(d));
    }

    // Put these in validation.cpp (inside namespace vl)

    static std::string to_lower_copy(std::string s)
    {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c)
                       { return static_cast<char>(std::tolower(c)); });
        return s;
    }

    // Extract variant suffix from task_id, e.g. "31-W2-a" -> "a", "31-W2-bb" -> "bb", "31-W2" -> ""
    static std::string parse_variant_suffix(const std::string &task_id)
    {
        // Accept letters-only suffix after "-W<digits>-"
        // Examples matched: "...-W2-a", "...-W12-bb"
        // Examples not matched (suffix=""): "...-W2", "weird"
        static const std::regex rgx(R"(.*-W\d+(?:-([A-Za-z]+))?$)");
        std::smatch m;
        if (std::regex_match(task_id, m, rgx))
        {
            if (m.size() >= 2 && m[1].matched)
            {
                return to_lower_copy(m[1].str());
            }
        }
        return "";
    }

    void validate_week_number_range(const std::vector<Task> &tasks)
    {
        struct Seen
        {
            int week;
            std::string task_id;
            std::size_t pos;
            bool is_intraweek;
            std::string variant; // parsed from task_id: "a", "b", "", etc.
        };

        // Group by schema, preserving input order
        std::map<int, std::vector<Seen>> by_schema;
        for (std::size_t i = 0; i < tasks.size(); ++i)
        {
            const auto &t = tasks[i];
            if (t.week < 1)
            {
                fail("Week out of range: task " + t.task_id +
                     " in schema " + std::to_string(t.schema_number) +
                     " has week " + std::to_string(t.week) + " (< 1) @pos " + std::to_string(i));
            }
            by_schema[t.schema_number].push_back(
                Seen{t.week, t.task_id, i, t.is_intraweek, parse_variant_suffix(t.task_id)});
        }

        for (auto &kv : by_schema)
        {
            const int schema = kv.first;
            const auto &seq = kv.second;

            // 1) Within each week, duplicates are allowed ONLY if all have a non-empty suffix (a/b/...)
            //    AND they are marked is_intraweek=true, AND suffixes are distinct.
            std::unordered_map<int, std::vector<const Seen *>> per_week;
            per_week.reserve(seq.size() * 2);
            for (const auto &s : seq)
                per_week[s.week].push_back(&s);

            for (const auto &wkv : per_week)
            {
                const int w = wkv.first;
                const auto &vec = wkv.second;
                if (vec.size() <= 1)
                    continue;

                // Multiple tasks in the same schema+week
                std::unordered_set<std::string> seen_suffixes;
                bool any_bad = false;
                std::ostringstream why;

                for (const Seen *sp : vec)
                {
                    const bool ok_suffix = !sp->variant.empty();
                    const bool ok_intra = sp->is_intraweek;

                    if (!ok_suffix)
                    {
                        any_bad = true;
                        why << " [" << sp->task_id << " @pos " << sp->pos << " has no -suffix]";
                    }
                    else if (!ok_intra)
                    {
                        any_bad = true;
                        why << " [" << sp->task_id << " @pos " << sp->pos << " not is_intraweek=true]";
                    }
                    else
                    {
                        if (!seen_suffixes.insert(sp->variant).second)
                        {
                            any_bad = true;
                            why << " [duplicate suffix '" << sp->variant
                                << "' for tasks in week " << w << "]";
                        }
                    }
                }

                if (any_bad)
                {
                    std::ostringstream msg;
                    msg << "Schema " << schema << " has multiple tasks in week " << w
                        << " but variants are invalid. Requirements: all must have "
                           "a distinct letter suffix (e.g., -a, -b) AND is_intraweek=true."
                        << why.str();
                    fail(msg.str());
                }
            }

            // 2) Weeks must be strictly increasing in input order, BUT allow repeats
            //    for intraweek variants in the same week.
            int last_week = 0;
            const Seen *last = nullptr;

            auto seq_str = [&seq]()
            {
                std::string s;
                s.reserve(seq.size() * 6);
                for (size_t i = 0; i < seq.size(); ++i)
                {
                    s += std::to_string(seq[i].week);
                    if (i + 1 < seq.size())
                        s += ",";
                }
                return s;
            }();

            for (const auto &s : seq)
            {
                if (last_week == 0)
                {
                    last_week = s.week;
                    last = &s;
                    continue;
                }

                if (s.week < last_week)
                {
                    fail("Non-increasing weeks in schema " + std::to_string(schema) + ": task " + s.task_id + " @pos " + std::to_string(s.pos) + " has week " + std::to_string(s.week) + " after " + last->task_id + " @pos " + std::to_string(last->pos) + " with week " + std::to_string(last->week) + ". Sequence: [" + seq_str + "]");
                }

                if (s.week == last_week)
                {
                    // Same week back-to-back in input order:
                    // Allowed only if BOTH are (is_intraweek && have suffix).
                    const bool curr_ok = s.is_intraweek && !s.variant.empty();
                    const bool prev_ok = last->is_intraweek && !last->variant.empty();
                    if (!(curr_ok && prev_ok))
                    {
                        fail("Duplicate week without valid intraweek variants in schema " + std::to_string(schema) + ": " + last->task_id + " (week " + std::to_string(last->week) + ", pos " + std::to_string(last->pos) + ") and " + s.task_id + " (week " + std::to_string(s.week) + ", pos " + std::to_string(s.pos) + "). Each duplicate must have a -suffix "
                                                                                                                                                                                                                                                                                                                           "(a/b/...) and is_intraweek=true. Sequence: [" +
                             seq_str + "]");
                    }
                }

                last_week = s.week;
                last = &s;
            }
        }
    }
    void validate_interval_positive(const std::vector<Task> &tasks)
    {
        for (const auto &t : tasks)
            if (t.frequency.value <= 0.0)
                fail("Task " + t.task_id + " has non-positive frequency value: " + std::to_string(t.frequency.value));
    }

    void validate_matrix_distance_range(const Matrix &matrix)
    {
        for (const auto &row : matrix)
            for (double v : row)
            {
                if (std::isnan(v))
                    fail("Distance matrix contains NaN values.");
                if (!std::isfinite(v))
                    fail("Distance matrix contains ∞ values.");
                if (v < 0.0)
                    fail("Distance matrix contains negative values.");
            }
    }

    void validate_matrix_task_alignment(const std::vector<Task> &tasks, const Matrix &matrix)
    {
        if (matrix.empty() || matrix.size() != matrix[0].size())
            fail("Matrix must be non-empty and square.");
        int max_idx = static_cast<int>(matrix.size()) - 1;
        std::unordered_set<int> ids;
        for (const auto &t : tasks)
            ids.insert(t.client_id);
        for (int cid : ids)
            if (cid < 0 || cid > max_idx)
                fail("Client ID " + std::to_string(cid) + " from tasks is out of bounds for matrix of size " + std::to_string(matrix.size()) + "x" + std::to_string(matrix.size()));
    }

    void validate_schema_canonical_structure(const std::vector<Task> &tasks)
    {
        std::map<int, std::vector<const Task *>> schemas;
        for (const auto &t : tasks)
            schemas[t.schema_number].push_back(&t);

        for (auto &[schema, group] : schemas)
        {
            if (group.empty())
                continue;
            const Task *base = group.front();
            for (size_t i = 1; i < group.size(); ++i)
            {
                const Task *tp = group[i];
                if (tp->client_id != base->client_id)
                    fail("Schema " + std::to_string(schema) + " has inconsistent client_id");
                if (tp->location.lat != base->location.lat || tp->location.lng != base->location.lng)
                    fail("Schema " + std::to_string(schema) + " has inconsistent location");
                if (tp->address.street != base->address.street ||
                    tp->address.city != base->address.city ||
                    tp->address.province != base->address.province)
                    fail("Schema " + std::to_string(schema) + " has inconsistent address");
            }
            std::vector<int> weeks;
            weeks.reserve(group.size());
            for (auto *tp : group)
                weeks.push_back(tp->week);
            if (!std::is_sorted(weeks.begin(), weeks.end()))
                fail("Schema " + std::to_string(schema) + " has unordered weeks");
        }
    }

    void validate_no_placeholder_matrix(const Matrix &matrix)
    {
        std::unordered_set<long long> uniq;
        for (const auto &row : matrix)
            for (double v : row)
                uniq.insert(static_cast<long long>(std::llround(v)));
        if (uniq.size() < 5)
            std::cerr << "⚠️ Matrix contains only " << uniq.size() << " unique values: may be placeholder or test matrix.\n";
    }

    void validate_task_spacing_within_schema(const std::vector<Task> &tasks)
    {
        std::map<int, std::map<int, std::vector<const Task *>>> M; // schema->week->tasks
        for (const auto &t : tasks)
            M[t.schema_number][t.week].push_back(&t);

        for (auto &[schema, weeks] : M)
        {
            for (auto &[week, group] : weeks)
            {
                if (group.size() < 2)
                    continue;

                for (size_t i = 0; i < group.size(); ++i)
                {
                    std::set<int> d1(group[i]->weekday_options.begin(), group[i]->weekday_options.end());
                    for (size_t j = i + 1; j < group.size(); ++j)
                    {
                        std::set<int> d2(group[j]->weekday_options.begin(), group[j]->weekday_options.end());
                        std::vector<int> inter;
                        std::set_intersection(d1.begin(), d1.end(), d2.begin(), d2.end(), std::back_inserter(inter));
                        if (!inter.empty())
                        {
                            std::ostringstream oss;
                            oss << "Schema " << schema << " week " << week << ": tasks "
                                << group[i]->task_id << " and " << group[j]->task_id
                                << " have overlapping day_options.";
                            fail(oss.str());
                        }
                        bool adjacent = false;
                        for (int a : d1)
                        {
                            for (int b : d2)
                                if (std::abs(a - b) == 1)
                                {
                                    adjacent = true;
                                    break;
                                }
                            if (adjacent)
                                break;
                        }
                        if (adjacent)
                        {
                            std::ostringstream oss;
                            oss << "Schema " << schema << " week " << week << ": tasks "
                                << group[i]->task_id << " and " << group[j]->task_id
                                << " have adjacent day_options.";
                            fail(oss.str());
                        }
                    }
                }

                std::set<int> all;
                for (auto *tp : group)
                    all.insert(tp->weekday_options.begin(), tp->weekday_options.end());
                int span = *all.rbegin() - *all.begin();
                int unique_days = static_cast<int>(all.size());
                int total_tasks = static_cast<int>(group.size());

                if (unique_days < total_tasks * 2)
                    std::cerr << "⚠️ Schema " << schema << " week " << week
                              << ": only " << unique_days << " unique day_options for "
                              << total_tasks << " tasks → assignment space is tight.\n";
                if (span < 3)
                    std::cerr << "⚠️ Schema " << schema << " week " << week
                              << ": day_options span only " << span << " days — may cause clustering.\n";
            }
        }
    }

    void validate_dayoption_overload(const std::vector<Task> &tasks,
                                     const Config &config,
                                     int planning_days)
    {
        const double max_daily = config.max_hours_per_route * config.max_routes_per_day * 60.0;
        std::map<int, double> load; // abs-day index -> minutes

        for (const auto &t : tasks)
        {
            if (t.weekday_options.empty())
                continue;
            double share = t.duration / static_cast<double>(t.weekday_options.size());
            int base = (t.week - 1) * 7;
            for (int wd : t.weekday_options)
            {
                if (wd < 0 || wd > 6)
                    continue;
                int day = base + wd;
                if (day < 0 || day >= planning_days)
                    continue;
                load[day] += share;
            }
        }

        bool any = false;
        for (auto &[day, minutes] : load)
        {
            if (minutes > max_daily)
            {
                if (!any)
                {
                    std::cerr << "⚠️ Potential overloads on specific days:\n";
                    any = true;
                }
                std::cerr << "  day#" << day << ": " << std::fixed << std::setprecision(1)
                          << minutes << " min > " << max_daily << " min capacity\n";
            }
        }
    }
}
