#pragma once

#include "solvers/Solver.hpp"
#include "solvers/SolverConfig.hpp"

class CompetitiveSolver : public Solver {
public:
    explicit CompetitiveSolver(SolverConfig config = {});
    Plan solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) override;
private:
    SolverConfig config_{};
};
