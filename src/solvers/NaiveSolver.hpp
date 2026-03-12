#pragma once

#include "solvers/Solver.hpp"

class NaiveSolver final : public Solver {
public:
    Plan solve(const Level& level, const State& initial_state) override;
};