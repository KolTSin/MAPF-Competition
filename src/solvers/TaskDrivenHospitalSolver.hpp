#pragma once

#include "solvers/Solver.hpp"

class TaskDrivenHospitalSolver final : public Solver {
public:
    Plan solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) override;
};
