#pragma once

#include "domain/Level.hpp"
#include "plan/Plan.hpp"
#include "state/State.hpp"

class Solver {
public:
    virtual ~Solver() = default;
    virtual Plan solve(const Level& level, const State& initial_state) = 0;
};