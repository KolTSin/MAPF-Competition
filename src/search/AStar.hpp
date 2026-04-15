#pragma once

#include "domain/Level.hpp"
#include "plan/AgentPlan.hpp"
#include "state/State.hpp"
#include "search/heuristics/Heuristic.hpp"

class AStar {
public:
    explicit AStar(const IHeuristic& heuristic):
        heuristic_(heuristic)
        {
        }

    AgentPlan search(const Level& level,
                       const State& initial_state,
                       int agent_id);

private:
    const IHeuristic& heuristic_;
};