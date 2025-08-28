#include "seed.h"
#include "third_party/json/single_include/nlohmann/json.hpp"
#include "seed_scoring.h"
#include <filesystem>
#include <fstream>


using json = nlohmann::json;
namespace fs = std::filesystem;

// ---- Seed JSON I/O ----

static json seed_task_to_json(const Task &t)
{
    // Writes back in the same “seed-shaped” format you described: 1 day_option
    json j;
    j["task_id"] = t.task_id;
    j["schema_number"] = t.schema_number;
    j["client_id"] = t.client_id;
    j["duration"] = t.duration; // minutes
    j["week"] = t.week;
    if (!t.day_options.empty())
    {
        j["day_options"] = json::array();
        j["day_options"].push_back({{"date", t.day_options.front().date},
                                    {"weekday", t.day_options.front().weekday}});
    }
    return j;
}

static void write_seed_json(const RankedSeed& rs, const fs::path& path) {
    json out = json::array();
    // Pre-reserve capacity on the underlying array container (if available)
    auto* arr = out.is_array() ? &out.get_ref<json::array_t&>() : nullptr;
    if (arr) arr->reserve(rs.seed.tasks.size());

    for (const auto& t : rs.seed.tasks) {
        out.push_back(seed_task_to_json(t));
    }

    fs::create_directories(path.parent_path());
    std::ofstream f(path);
    if (!f) throw std::runtime_error("Cannot open " + path.string() + " for writing.");
    f << out.dump(2) << "\n";
}

static std::vector<Task> load_seed_shaped_tasks_from_json(const json &tasks_json)
{
    if (!tasks_json.is_array())
        throw std::runtime_error("seed_tasks.json must be an array.");
    std::vector<Task> tasks;
    tasks.reserve(tasks_json.size());
    for (const auto &t : tasks_json)
    {
        Task x;
        x.task_id = t.at("task_id").get<std::string>();
        x.schema_number = t.at("schema_number").get<int>();
        x.client_id = t.at("client_id").get<int>();
        x.duration = (t.contains("duration_seconds"))
                         ? std::llround(t.at("duration_seconds").get<double>() / 60.0)
                         : t.at("duration").get<int>();
        x.week = t.at("week").get<int>();
        // Expect exactly one day_option; if multiple/none, we still accept and pick the first/leave empty.
        if (t.contains("day_options") && t["day_options"].is_array() && !t["day_options"].empty())
        {
            auto &opt = t["day_options"].front();
            Task::DayOption d;
            d.date = opt.value("date", "");
            d.weekday = opt.value("weekday", 0);
            x.day_options.push_back(std::move(d));
        }
        tasks.push_back(std::move(x));
    }
    return tasks;
}
