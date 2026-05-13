// include/config/SearchConfig.hpp
#pragma once

#include <string>
#include <optional>

enum class SolverType {
    NAIVE,
    SEQUENTIAL,
    SPACETIME_ASTAR,
    CBS,
    COMPETITIVE,
    LNS_HTN
};

enum class HeuristicType {
    GOAL_COUNT,
    AGENT_GOAL_DISTANCE,
    BOX_GOAL_DISTANCE
    // add more here
};

struct SearchConfig {
    SolverType solver = SolverType::NAIVE;
    std::string heuristic_name = "bgd";

    bool verbose = true;
    bool help = false;

    // optional extras
    std::optional<int> max_expansions;
    std::optional<int> max_depth;
};