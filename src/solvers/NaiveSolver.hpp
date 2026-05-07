#pragma once

#include "solvers/Solver.hpp"

// Baseline solver that plans without sophisticated coordination.
class NaiveSolver final : public Solver {
public:
    Plan solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) override;
};