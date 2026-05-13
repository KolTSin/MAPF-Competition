#pragma once

#include "solvers/Solver.hpp"
#include "solvers/SolverConfig.hpp"

// HTN + reservation planner with large-neighborhood search repair and a
// one-step priority-inheritance-style fallback for stuck execution frontiers.
class LNSHTNSolver : public Solver {
public:
    explicit LNSHTNSolver(SolverConfig config = {});
    Plan solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) override;
private:
    SolverConfig config_{};
};
