#pragma once

#include "actions/Action.hpp"
#include "domain/Level.hpp"
#include "state/State.hpp"

#include <vector>

struct Successor {
    Action action;
    State next_state;
};

class SuccessorGenerator {
public:
    // Expand all legal successors for one agent
    static void expand_agent(
        const Level& level,
        const State& state,
        int agent_id,
        int time,
        std::vector<Successor>& out);
};