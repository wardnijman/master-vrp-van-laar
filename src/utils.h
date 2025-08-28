#pragma once
#include <chrono>
#include "third_party/json/single_include/nlohmann/json.hpp"
#include <sstream>

using json = nlohmann::json;

// Returns current time in milliseconds since epoch
static inline long long NowMillis() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(
               steady_clock::now().time_since_epoch()
           ).count();
}


// ---------- small date helpers ----------
static std::tm parse_ymd(const std::string& ymd) {
    std::tm tm{}; tm.tm_isdst = -1;
    std::istringstream ss(ymd);
    ss >> std::get_time(&tm, "%Y-%m-%d");
    if (ss.fail()) throw std::runtime_error("Bad date: " + ymd);
    return tm;
}
static std::string ymd_add_days(const std::string& ymd, int d) {
    std::tm tm = parse_ymd(ymd);
    auto t = std::chrono::system_clock::from_time_t(std::mktime(&tm));
    t += std::chrono::hours(24 * d);
    std::time_t tt = std::chrono::system_clock::to_time_t(t);
    std::ostringstream out; out << std::put_time(std::localtime(&tt), "%Y-%m-%d");
    return out.str();
}
static int ymd_cmp(const std::string& a, const std::string& b) {
    // string compare won't work across months; parse
    std::tm ta = parse_ymd(a), tb = parse_ymd(b);
    auto xa = std::mktime(&ta), xb = std::mktime(&tb);
    if (xa < xb) return -1; if (xa > xb) return 1; return 0;
}
static bool ymd_in_range(const std::string& y, const std::string& lo, const std::string& hi) {
    return ymd_cmp(y, lo) >= 0 && ymd_cmp(y, hi) <= 0;
}
// 0=Sun..6=Sat → 1..7 with Mon=1, Sun=7
static int weekday1_from_date(const std::string& ymd) {
    std::tm tm = parse_ymd(ymd);
    std::time_t tt = std::mktime(&tm);
    std::tm* lt = std::localtime(&tt);
    int w = lt ? lt->tm_wday : 0; // 0=Sun
    return (w == 0) ? 7 : w;      // Mon=1..Sun=7
}

// ---------- extract earliest date in tasks ----------
static std::string find_min_date(const json& tasks_json) {
    std::string min_d;
    bool have = false;
    for (const auto& t : tasks_json) {
        if (!t.contains("day_options")) continue;
        for (const auto& opt : t["day_options"]) {
            if (!opt.contains("date")) continue;
            const std::string d = opt["date"].get<std::string>();
            if (!have || ymd_cmp(d, min_d) < 0) { min_d = d; have = true; }
        }
    }
    if (!have) throw std::runtime_error("No day_options/date in tasks.json");
    return min_d;
}