#pragma once

#include "domain/Level.hpp"
#include "plan/Plan.hpp"
#include "state/State.hpp"
#include "search/heuristics/Heuristic.hpp"

// Strategy interface for all complete planning algorithms.
class Solver {
public:
    virtual ~Solver() = default;
    virtual Plan solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) = 0;
};