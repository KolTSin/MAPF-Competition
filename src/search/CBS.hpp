#pragma once

#include "actions/Action.hpp"
#include "domain/Level.hpp"
#include "domain/Position.hpp"
#include "plan/ReservationTable.hpp"
#include "plan/AgentPlan.hpp"
#include "search/heuristics/Heuristic.hpp"

#include <vector>

class CBS {
public:
    explicit CBS(const IHeuristic& heuristic):
        heuristic_(heuristic)
        {
        }

    AgentPlan search(
        const Level& level,
        const State& initial_state,
        const int agent,
        const int max_time,
        const ReservationTable& reservations
    );
private:
    const IHeuristic& heuristic_;
};