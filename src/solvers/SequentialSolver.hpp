#pragma once

#include "solvers/Solver.hpp"

// Plans agents/tasks one after another to reduce simultaneous conflicts.
class SequentialSolver final : public Solver {
public:
    Plan solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) override;
};