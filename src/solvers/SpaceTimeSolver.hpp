#pragma once

#include "solvers/Solver.hpp"

// Solver wrapper around reservation-aware space-time A*.
class SpaceTimeSolver final : public Solver {
public:
    Plan solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) override;
};