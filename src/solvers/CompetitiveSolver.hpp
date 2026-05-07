#pragma once

#include "solvers/Solver.hpp"
#include "solvers/SolverConfig.hpp"

// HTN/reservation-based solver aimed at multi-agent competition levels.
//
// Input to solve(): a parsed Level, the current State, and the shared Solver
// interface's heuristic object. Output: a Plan containing synchronized
// JointActions for the competition client. Internally, this solver makes the
// input/output flow readable by processing the level in repeated waves:
// generate tasks -> schedule a safe prefix -> simulate that prefix -> repeat.
class CompetitiveSolver : public Solver {
public:
    explicit CompetitiveSolver(SolverConfig config = {});
    Plan solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) override;
private:
    // Runtime knobs such as planning budget and HTN trace verbosity.
    SolverConfig config_{};
};
