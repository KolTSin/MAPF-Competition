#pragma once

#include "actions/Action.hpp"
#include "domain/Level.hpp"
#include "domain/Position.hpp"
#include "plan/ReservationTable.hpp"
#include "state/State.hpp"

#include <vector>

// One legal transition from a state for a single acting agent.
struct Successor {
    Action action;
    State next_state;
};

// Lightweight transition for task-level agent-only planning.
struct AgentTaskSuccessor {
    Action action;
    Position next_agent;
    int next_time{0};
};

// Compact state used by task-level single-box transport planning.
struct BoxTransportSearchState {
    Position agent{};
    Position box{};
    int time{0};
};

// Lightweight transition for task-level single-box transport planning.
struct BoxTransportSuccessor {
    Action action;
    BoxTransportSearchState next{};
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

    // Expand agent-only task planning over position/time states. Static boxes and
    // other agents are treated as obstacles; reservations filter space-time moves.
    static void expand_agent_task(
        const Level& level,
        const State& state,
        int agent_id,
        Position current,
        int relative_time,
        int start_time,
        const ReservationTable& reservations,
        std::vector<AgentTaskSuccessor>& out);

    // Expand single-active-box task planning over (agent, box, time). Other boxes
    // and agents are static obstacles, while reservations filter both agent and
    // active-box transitions.
    static void expand_box_transport(
        const Level& level,
        const State& state,
        int agent_id,
        char box_id,
        Position original_active_box,
        const BoxTransportSearchState& current,
        int start_time,
        const ReservationTable& reservations,
        std::vector<BoxTransportSuccessor>& out);
};
