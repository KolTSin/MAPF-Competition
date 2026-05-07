#pragma once

#include "actions/Action.hpp"
#include "domain/Level.hpp"
#include "state/State.hpp"

#include <vector>

// One legal transition from a state for a single acting agent.
struct Successor {
    Action action;
    State next_state;
};

class SuccessorGenerator {
public:
    // Try every primitive action for one agent and append only legal outcomes.
    // The output vector is reused by callers, so this function clears it first.
    static void expand_agent(
        const Level& level,
        const State& state,
        int agent_id,
        int time,
        std::vector<Successor>& out);
};
