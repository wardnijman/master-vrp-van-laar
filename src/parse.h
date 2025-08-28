#include <string>
#include "third_party/json/single_include/nlohmann/json.hpp"
#include <sstream>
#include <ctime>
#include "types.h"
#include "schedule_state.h"
#include <iostream>
#include <unordered_set>

using json = nlohmann::json;

static char deduce_suffix(const std::string &task_id, const json &t)
{
    if (t.contains("suffix") && t["suffix"].is_string() && !t["suffix"].get<std::string>().empty())
    {
        return t["suffix"].get<std::string>()[0];
    }
    // Heuristic from IDs like "814a-W2" or "31b-W1": find a letter just before "-W"
    auto pos = task_id.find("-W");
    if (pos != std::string::npos && pos > 0)
    {
        char c = task_id[pos - 1];
        if (std::isalpha(static_cast<unsigned char>(c)))
            return c;
    }
    return '\0';
}

static int weekday_from_date(const std::string &ymd)
{
    // returns 0..6 in the C tm_wday convention: 0=Sunday ... 6=Saturday
    std::tm tm{};
    tm.tm_isdst = -1;
    std::istringstream ss(ymd);
    ss >> std::get_time(&tm, "%Y-%m-%d");
    if (ss.fail())
        return 0;
    std::time_t tt = std::mktime(&tm);
    if (tt == -1)
        return 0;
    std::tm *lt = std::localtime(&tt);
    if (!lt)
        return 0;
    return lt->tm_wday; // 0..6
}

static std::vector<int> parse_weekday_options_from_task(const json &t)
{
    std::vector<int> out;
    if (t.contains("weekday_options") && t["weekday_options"].is_array())
    {
        for (const auto &v : t["weekday_options"])
        {
            if (v.is_number_integer())
            {
                int wd = v.get<int>();
                if (0 <= wd && wd <= 6)
                    out.push_back(wd);
            }
        }
    }
    return out;
}

static int to_minutes_from_seconds(double sec)
{
    // round to nearest minute; constrain to >= 0
    if (!std::isfinite(sec) || sec < 0)
        return 0;
    return static_cast<int>(std::llround(sec / 60.0));
}

static TimeMatrix build_time_matrix_minutes(const json &matrix_json)
{
    // Accept either 2D array of seconds or object with "n" + "data"
    TimeMatrix TM{};
    if (matrix_json.is_array() && !matrix_json.empty() && matrix_json[0].is_array())
    {
        const int n = static_cast<int>(matrix_json.size());
        TM.n = n;
        TM.m.resize(static_cast<size_t>(n) * n);
        for (int i = 0; i < n; ++i)
        {
            const auto &row = matrix_json[i];
            if (!row.is_array() || static_cast<int>(row.size()) != n)
            {
                throw std::runtime_error("Matrix row size mismatch.");
            }
            for (int j = 0; j < n; ++j)
            {
                double sec = row[j].get<double>();
                TM.m[i * n + j] = to_minutes_from_seconds(sec);
            }
        }
        return TM;
    }
    if (matrix_json.is_object() && matrix_json.contains("n") && matrix_json.contains("data"))
    {
        int n = matrix_json["n"].get<int>();
        const auto &data = matrix_json["data"];
        if (!data.is_array() || static_cast<int>(data.size()) != n * n)
        {
            throw std::runtime_error("Matrix 'data' must be flat n*n array.");
        }
        TM.n = n;
        TM.m.resize(static_cast<size_t>(n) * n);
        for (int idx = 0; idx < n * n; ++idx)
        {
            double sec = data[idx].get<double>();
            TM.m[idx] = to_minutes_from_seconds(sec);
        }
        return TM;
    }
    throw std::runtime_error("Unsupported matrix JSON format.");
}

static std::vector<TaskRef> build_tasks(const json& tasks_json, int& out_max_week) {
    if (!tasks_json.is_array()) throw std::runtime_error("tasks_json must be an array.");
    std::vector<TaskRef> tasks;
    tasks.reserve(tasks_json.size());
    int max_week = 0;

    int empty_opt_count = 0;
    for (const auto& t : tasks_json) {
        TaskRef r{};
        r.task_id        = t.at("task_id").get<std::string>();
        r.schema_number  = t.at("schema_number").get<int>();
        r.week           = t.at("week").get<int>();             // keep provided week
        r.client_idx     = t.at("client_id").get<int>();        // index into matrix (0 = depot)
        r.suffix         = deduce_suffix(r.task_id, t);

        // Duration: allow "duration" (minutes) or "duration_seconds"
        if (t.contains("duration_seconds")) {
            r.service_minutes = to_minutes_from_seconds(t["duration_seconds"].get<double>());
        } else if (t.contains("duration")) {
            r.service_minutes = t["duration"].get<int>(); // assume already minutes
        } else {
            throw std::runtime_error("Task " + r.task_id + " missing duration/duration_seconds.");
        }

        // Derive weekday_options from "day_options" dates (preferred) OR from explicit weekday_options if present.
        std::vector<int> opts;
        if (t.contains("day_options") && t["day_options"].is_array()) {
            std::unordered_set<int> seen;
            for (const auto& opt : t["day_options"]) {
                if (!opt.is_object() || !opt.contains("date")) continue;
                const std::string date = opt["date"].get<std::string>();
                int wd = weekday_from_date(date); // 0=Sun..6=Sat
                if (seen.insert(wd).second) opts.push_back(wd);
            }
        }
        // If explicit weekday_options exists, merge it in too (dedup)
        if (t.contains("weekday_options") && t["weekday_options"].is_array()) {
            std::unordered_set<int> seen(opts.begin(), opts.end());
            for (const auto& v : t["weekday_options"]) {
                if (!v.is_number_integer()) continue;
                int wd = v.get<int>();
                if (0 <= wd && wd <= 6 && seen.insert(wd).second) opts.push_back(wd);
            }
        }

        if (opts.empty()) {
            // Fallback: allow all days (very permissive; SA will shape it), but warn once every many.
            // If you prefer stricter behavior, throw here instead.
            ++empty_opt_count;
            opts = {0,1,2,3,4,5,6};
        }
        // Sort for determinism
        std::sort(opts.begin(), opts.end());
        r.weekday_options = std::move(opts);

        if (r.week > max_week) max_week = r.week;
        tasks.push_back(std::move(r));
    }

    if (empty_opt_count > 0) {
        std::cerr << "⚠️  Derived weekday_options were empty for " << empty_opt_count
                  << " tasks; defaulted to all 7 weekdays.\n";
    }

    out_max_week = max_week;
    return tasks;
}

static json serialize_calendar(const ScheduleState &S)
{
    json days = json::array();
    for (const auto &kv : S.calendar)
    {
        int day_index = kv.first;
        const auto &day_tasks = kv.second;
        json arr = json::array();
        for (const auto &tr : day_tasks)
            arr.push_back(tr.task_id);
        days.push_back({{"day_index", day_index},
                        {"tasks", arr}});
    }
    return days;
}