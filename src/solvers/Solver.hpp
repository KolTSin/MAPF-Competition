#pragma once

#include "domain/Level.hpp"
#include "plan/Plan.hpp"

class Solver {
public:
    virtual ~Solver() = default;
    virtual Plan solve(const Level& level) = 0;
};