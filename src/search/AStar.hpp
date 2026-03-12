#pragma once

#include "domain/Level.hpp"
#include "plan/Plan.hpp"
#include "state/State.hpp"

class AStar {
public:
    static Plan search(const Level& level,
                       const State& initial_state,
                       int agent_id);
};