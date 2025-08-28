// main.cpp — updated to use the new first-schedule “middleman” when --seed-only is set.
// Note: diversity gating in the middleman currently uses an assignment-based distance
// (task→(day,route) equality ratio). If you want Hungarian/Jaccard route matching later,
// we can swap it in the middleman without touching main.cpp.

#include "validation.h"
#include "utils.h"
#include "day_solver.h"
#include "parse.h"
#include "seed_io.h"

// SA 2-level headers
#include "sa.h"
#include "types.h"
#include "schedule_state.h"

// SEED portfolio (kept for the non-middleman path)
#include "seed_scoring.h"
#include "seed_generators.h"

// First-schedule (2-week rollout) solver
#include "first_schedule_solver.h"
#include "first_solution_parser.h"

// NEW: middleman that harvests 26v runs, eliminates to 25, and warm-start polishes
#include "first_schedule_middleman.h"

#include <fstream>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <cassert>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <string>
#include <sstream>
#include <ctime>
#include <filesystem>
#include <algorithm>

#include "third_party/json/single_include/nlohmann/json.hpp"
#include "tests.h"

using json = nlohmann::json;
namespace fs = std::filesystem;

static bool g_verbose = false;

// ---------- CLI ----------
static void print_usage()
{
    std::cerr
        << "Usage: ./sa_solver tasks.json time_matrix_seconds.json [options]\n"
        << "Options:\n"
        << "  --first-weeks=W                 First-schedule window length in weeks (default 2)\n"
        << "  --cache-matrix                  Use matrix cache (validation path only)\n"
        << "  -v, --verbose                   Verbose logs\n"
        << "  --iters=N                       SA iterations (default 10000)\n"
        << "  --seed=N                        RNG seed (wired into SA)\n"
        << "  --explore-seconds=S             Per-day time during seed scoring (default 2)\n"
        << "  --intensify-seconds=S           Per-day time during SA intensification (default 60)\n"
        << "  --gens=g1,g2,...                Seed generators to run "
           "(default: passthrough,two_week_rollout,capacity_balanced,sweep)\n"
        << "  --topk=K                        Keep top-K seeds (default 3)\n"
        << "  --seeds-out=DIR                 Write seeds to DIR (default: ./out_seeds)\n"
        << "  --seed-only                     Build seeds via middleman and exit (skip SA)\n"
        << "  --mm-runs=N                     Middleman: number of relaxed 26-vehicle runs (default 16)\n"
        << "  --mm-relaxed-vehicles=N         Middleman: relaxed vehicles_per_day (default 26)\n"
        << "  --mm-target-vehicles=N          Middleman: target vehicles_per_day (default 25)\n"
        << "  --mm-relaxed-seconds=S          Middleman: time limit per relaxed run (default 30)\n"
        << "  --mm-polish-seconds=S           Middleman: time limit per 25v polish (default 15)\n"
        << "  --mm-diversity=F                Middleman: diversity threshold (default 0.10)\n"
        << "  --mm-seed-base=N                Middleman: base RNG seed added per run index (default 12345)\n";
}

// ---------- MAIN ----------
int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        print_usage();
        return 1;
    }

    // Defaults
    int first_weeks = 2;
    bool use_matrix_cache = false;
    int iters = 10000;
    int seed = 42;
    int explore_seconds = 2;
    int intensify_seconds = 60;
    int top_k = 3;
    bool seed_only = false;
    fs::path seeds_out_dir = "./out_seeds";
    std::vector<std::string> generators = {
        "passthrough", "two_week_rollout", "capacity_balanced", "sweep"};

    // Middleman defaults
    int mm_runs = 16;
    int mm_relaxed_vehicles = 26;
    int mm_target_vehicles = 25;
    int mm_relaxed_seconds = 30;
    int mm_polish_seconds = 15;
    double mm_diversity = 0.10;
    int mm_seed_base = 12345;

    // Parse CLI flags
    for (int i = 3; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg.rfind("--first-weeks=", 0) == 0)
        {
            first_weeks = std::max(1, std::stoi(arg.substr(14)));
        }
        else if (arg == "--cache-matrix")
        {
            std::cout << "Using matrix cache\n";
            use_matrix_cache = true;
        }
        else if (arg == "--verbose" || arg == "-v")
        {
            g_verbose = true;
        }
        else if (arg.rfind("--iters=", 0) == 0)
        {
            iters = std::stoi(arg.substr(8));
        }
        else if (arg.rfind("--seed=", 0) == 0)
        {
            seed = std::stoi(arg.substr(7));
        }
        else if (arg.rfind("--explore-seconds=", 0) == 0)
        {
            explore_seconds = std::stoi(arg.substr(18));
        }
        else if (arg.rfind("--intensify-seconds=", 0) == 0)
        {
            intensify_seconds = std::stoi(arg.substr(20));
        }
        else if (arg.rfind("--gens=", 0) == 0)
        {
            generators.clear();
            std::string s = arg.substr(7);
            size_t pos = 0;
            while (pos != std::string::npos)
            {
                size_t next = s.find(',', pos);
                generators.emplace_back(s.substr(pos, next == std::string::npos ? next : (next - pos)));
                if (next == std::string::npos)
                    break;
                pos = next + 1;
            }
        }
        else if (arg.rfind("--topk=", 0) == 0)
        {
            top_k = std::max(1, std::stoi(arg.substr(7)));
        }
        else if (arg.rfind("--seeds-out=", 0) == 0)
        {
            seeds_out_dir = fs::path(arg.substr(12));
        }
        else if (arg == "--seed-only")
        {
            seed_only = true;
        }
        // Middleman options
        else if (arg.rfind("--mm-runs=", 0) == 0)
        {
            mm_runs = std::max(1, std::stoi(arg.substr(10)));
        }
        else if (arg.rfind("--mm-relaxed-vehicles=", 0) == 0)
        {
            mm_relaxed_vehicles = std::max(1, std::stoi(arg.substr(22)));
        }
        else if (arg.rfind("--mm-target-vehicles=", 0) == 0)
        {
            mm_target_vehicles = std::max(1, std::stoi(arg.substr(21)));
        }
        else if (arg.rfind("--mm-relaxed-seconds=", 0) == 0)
        {
            mm_relaxed_seconds = std::max(1, std::stoi(arg.substr(21)));
        }
        else if (arg.rfind("--mm-polish-seconds=", 0) == 0)
        {
            mm_polish_seconds = std::max(1, std::stoi(arg.substr(20)));
        }
        else if (arg.rfind("--mm-diversity=", 0) == 0)
        {
            mm_diversity = std::stod(arg.substr(15));
        }
        else if (arg.rfind("--mm-seed-base=", 0) == 0)
        {
            mm_seed_base = std::stoi(arg.substr(15));
        }
        else
        {
            std::cerr << "Unknown option: " << arg << "\n";
            print_usage();
            return 1;
        }
    }

    // Load tasks & matrix
    json tasks_json;
    {
        std::ifstream f(argv[1]);
        if (!f)
        {
            std::cerr << "Cannot open tasks file: " << argv[1] << "\n";
            return 1;
        }
        try
        {
            f >> tasks_json;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to parse tasks JSON: " << e.what() << "\n";
            return 1;
        }
    }
    json matrix_json;
    {
        std::ifstream f(argv[2]);
        if (!f)
        {
            std::cerr << "Cannot open matrix file: " << argv[2] << "\n";
            return 1;
        }
        try
        {
            f >> matrix_json;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to parse matrix JSON: " << e.what() << "\n";
            return 1;
        }
    }

    // Baseline config for validation & day caps
    json config = {
        {"MAX_HOURS_PER_DAY", 9},
        {"VEHICLES_PER_DAY", 30},
        {"TIME_LIMIT_SECONDS", 30},
        {"USE_MATRIX_CACHE", use_matrix_cache}};

    // Validate full input (as usual)
    try
    {
        std::cout << "🔍 Validating...\n";
        vl::validate_all(tasks_json, matrix_json, config);
        std::cout << "✅ Input validated.\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "Validation error: " << e.what() << "\n";
        return 1;
    }

    // -------- FIRST SCHEDULE WINDOW (2 weeks by default) --------
    const std::string min_date = find_min_date(tasks_json);
    const std::string win_end = ymd_add_days(min_date, first_weeks * 7 - 1);
    json window_tasks = filter_tasks_to_window(tasks_json, min_date, win_end);

    if (g_verbose) {
        std::cout << "Window: [" << min_date << " .. " << win_end << "]\n";
        // std::cout << "window_tasks: " << window_tasks.dump(2) << "\n"; // uncomment if you really want the dump
    }

    // If we only want to generate seeds via the middleman, do it now and exit.
    if (seed_only)
    {
        fsmm::Config mmcfg;
        mmcfg.runs = mm_runs;
        mmcfg.vehicles_relaxed = mm_relaxed_vehicles;
        mmcfg.vehicles_target  = mm_target_vehicles;
        mmcfg.time_limit_relaxed_s = mm_relaxed_seconds;
        mmcfg.time_limit_polish_s  = mm_polish_seconds;
        mmcfg.diversity_threshold  = mm_diversity;
        mmcfg.random_seed_base     = mm_seed_base;
        mmcfg.verbose              = g_verbose;

        std::cout << "🧪 Middleman seeding: runs=" << mmcfg.runs
                  << " relaxed=" << mmcfg.vehicles_relaxed
                  << " target="  << mmcfg.vehicles_target
                  << " tlx="     << mmcfg.time_limit_relaxed_s
                  << " tlp="     << mmcfg.time_limit_polish_s
                  << " div="     << mmcfg.diversity_threshold
                  << " base-seed=" << mmcfg.random_seed_base
                  << "\n";

        std::vector<json> seeds = fsmm::build_seeds(window_tasks, matrix_json, config, mmcfg);

        fs::create_directories(seeds_out_dir);
        int written = 0;
        for (size_t i = 0; i < seeds.size(); ++i)
        {
            std::ostringstream fn;
            fn << "mm_seed_" << std::setw(2) << std::setfill('0') << (i + 1) << ".json";
            const fs::path path = seeds_out_dir / fn.str();
            std::ofstream out(path);
            if (!out) {
                std::cerr << "Failed to write: " << path.string() << "\n";
                continue;
            }
            out << seeds[i].dump(2) << "\n";
            ++written;
            if (g_verbose) std::cout << "  wrote " << path.string() << "\n";
        }
        std::cout << "💾 Middleman wrote " << written << " seed(s) to: " << seeds_out_dir.string() << "\n";
        std::cout << "✅ --seed-only set; exiting after seed generation.\n";
        return 0;
    }

    // ---------- ORIGINAL seed portfolio path (kept for SA) ----------
    // Solve the first-schedule window once to extract canonicals
    json first_sol;
    try
    {
        std::cout << "🧭 First-schedule solve for window [" << min_date << " .. " << win_end << "]\n";
        first_sol = first_schedule_solver::solve_problem(window_tasks, matrix_json, config);
        std::cout << "✅ First-schedule solution obtained.\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "First-schedule solver error: " << e.what() << "\n";
        return 1;
    }

    // Extract canonical weekday per schema from the first-schedule result
    std::unordered_map<int, int> canonical;
    if (first_sol.contains("canonical_weekday_by_schema") &&
        first_sol["canonical_weekday_by_schema"].is_object())
    {
        for (auto it = first_sol["canonical_weekday_by_schema"].begin();
             it != first_sol["canonical_weekday_by_schema"].end(); ++it)
        {
            int schema = std::stoi(it.key());
            int wd1 = it.value().get<int>();
            canonical[schema] = wd1;
        }
    }
    else
    {
        canonical = canonical_from_first_schedule(first_sol, tasks_json);
    }

    if (g_verbose)
    {
        std::cout << "Canonical weekdays (schema -> wd1..7): ";
        int shown = 0;
        for (const auto &kv : canonical)
        {
            if (shown++ >= 12) { std::cout << "..."; break; }
            std::cout << kv.first << ":" << kv.second << " ";
        }
        std::cout << "\n";
    }

    // Build a single-day-option seed for the FULL horizon using the canonicals
    std::vector<Task> base_seed_tasks = build_seed_from_canonical(tasks_json, canonical);

    // Light balancing for weeks not covered by the first window (service-only heuristic)
    balance_seed_service_load(base_seed_tasks, tasks_json,
                              config["VEHICLES_PER_DAY"].get<int>(),
                              config["MAX_HOURS_PER_DAY"].get<int>() * 60);

    // -------- Build & rank seed portfolio on top of this base assignment --------
    register_default_seed_generators();

    // Build TimeMatrix minutes for seed scoring
    TimeMatrix TM = build_time_matrix_minutes(matrix_json);

    // Seed scoring config uses real day VRP (short time) for fairness across seeds
    EvalConfig eval_cfg;
    eval_cfg.max_routes_per_day = config["VEHICLES_PER_DAY"].get<int>();
    eval_cfg.max_hours_per_route = config["MAX_HOURS_PER_DAY"].get<int>();
    eval_cfg.matrix = &TM;
    eval_cfg.day_cfg = {};
    eval_cfg.day_cfg.vehicles = eval_cfg.max_routes_per_day;
    eval_cfg.day_cfg.route_limit_minutes = eval_cfg.max_hours_per_route * 60;
    eval_cfg.day_cfg.soft_upper_cost = 1;
    eval_cfg.day_cfg.span_coeff = 0;
    eval_cfg.explore_params = {};
    eval_cfg.explore_params.time_limit_seconds = std::max(1, explore_seconds);
    eval_cfg.explore_params.use_warm_start = false;
    eval_cfg.explore_params.log_search = false;

    // Which generators to run
    std::cout << "🧩 Building seeds from base assignment: ";
    for (size_t i = 0; i < generators.size(); ++i)
        std::cout << generators[i] << (i + 1 < generators.size() ? "," : "");
    std::cout << "  (topk=" << top_k << ")\n";

    // Run portfolio
    auto ranked = build_and_rank_seeds(generators, base_seed_tasks, eval_cfg, top_k);
    if (ranked.empty())
    {
        std::cerr << "No seeds produced.\n";
        return 1;
    }

    print_ranked_seeds(ranked);

    // Write seeds to disk
    fs::create_directories(seeds_out_dir);
    for (size_t i = 0; i < ranked.size(); ++i)
    {
        const auto &rs = ranked[i];
        std::ostringstream fn;
        fn << std::setw(2) << std::setfill('0') << (i + 1) << "_" << rs.seed.name << ".json";
        write_seed_json(rs, seeds_out_dir / fn.str());
    }
    std::cout << "💾 Wrote " << ranked.size() << " seed(s) to: " << seeds_out_dir.string() << "\n";

    // -------- SA path (unchanged) --------
    int max_week = 0;
    std::vector<TaskRef> tasks_ref = build_tasks(tasks_json, max_week);

    SA_Config cfg;
    cfg.total_weeks = max_week;
    cfg.day_cfg.vehicles = config["VEHICLES_PER_DAY"].get<int>();
    cfg.day_cfg.route_limit_minutes = config["MAX_HOURS_PER_DAY"].get<int>() * 60;
    cfg.day_cfg.exploration_seconds = std::max(1, explore_seconds);
    cfg.day_cfg.intensify_seconds = std::max(cfg.day_cfg.exploration_seconds, intensify_seconds);
    cfg.W.w_dev = 100;
    cfg.W.w_space = 120;
    cfg.W.w_ot = 7;
    cfg.W.w_vu = 1;
    cfg.T0 = 5.0;
    cfg.alpha = 0.995;
    cfg.iters = iters;
    cfg.reheats_every = 3000;
    cfg.verbose = g_verbose;
    cfg.seed = seed;

    // Quick warm-start smoke test
    assert_warmup_working(tasks_ref, TM, cfg);

    if (g_verbose)
    {
        std::cout << std::boolalpha
                  << "Config:\n"
                  << "  weeks=" << cfg.total_weeks << "\n"
                  << "  vehicles/day=" << cfg.day_cfg.vehicles << "\n"
                  << "  route_limit_minutes=" << cfg.day_cfg.route_limit_minutes << "\n"
                  << "  iters=" << cfg.iters << "\n"
                  << "  explore_seconds=" << cfg.day_cfg.exploration_seconds << "\n"
                  << "  intensify_seconds=" << cfg.day_cfg.intensify_seconds << "\n"
                  << "  verbose=" << g_verbose << "\n"
                  << "  seed=" << seed << "\n";
    }

    SA_Result res;
    try
    {
        std::cout << "🚀 Running SA 2-level optimizer...\n";
        res = run_sa(tasks_ref, TM, cfg);
        std::cout << "🏁 SA done.\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "Solve error: " << e.what() << "\n";
        return 1;
    }

    json out = {
        {"meta", {{"vehicles_per_day", cfg.day_cfg.vehicles}, {"route_limit_minutes", cfg.day_cfg.route_limit_minutes}, {"iters", cfg.iters}, {"explore_seconds", cfg.day_cfg.exploration_seconds}, {"intensify_seconds", cfg.day_cfg.intensify_seconds}, {"seed", cfg.seed}}},
        {"best_cost", {{"routes_minutes", res.best_cost.routes_minutes}, {"weekday_deviation", res.best_cost.weekday_dev}, {"variant_spacing", res.best_cost.spacing}, {"overtime_and_vehicle", res.best_cost.ot_vu}, {"total", res.best_cost.total()}}},
        {"calendar", serialize_calendar(res.best_state)}};

    std::cout << out.dump(2) << "\n";
    return 0;
}
