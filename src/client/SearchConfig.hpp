// include/config/SearchConfig.hpp
#pragma once

#include <string>
#include <optional>

enum class SolverType {
    NAIVE,
    SEQUENTIAL,
    SPACETIME_ASTAR,
    TASK_DRIVEN_HOSPITAL
};

enum class HeuristicType {
    GOAL_COUNT,
    AGENT_GOAL_DISTANCE,
    BOX_GOAL_DISTANCE
    // add more here
};

struct SearchConfig {
    SolverType solver = SolverType::NAIVE;
    std::string heuristic_name = "agd";

    bool verbose = false;
    bool help = false;

    // optional extras
    std::optional<int> max_expansions;
    std::optional<int> max_depth;
};
