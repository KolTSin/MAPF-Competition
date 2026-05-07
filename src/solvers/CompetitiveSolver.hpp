#pragma once

#include "solvers/Solver.hpp"
#include "solvers/SolverConfig.hpp"

// HTN/reservation-based solver aimed at multi-agent competition levels.
class CompetitiveSolver : public Solver {
public:
    explicit CompetitiveSolver(SolverConfig config = {});
    Plan solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) override;
private:
    SolverConfig config_{};
};
