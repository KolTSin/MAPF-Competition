#pragma once

#include "domain/Level.hpp"
#include "plan/Plan.hpp"
#include "state/State.hpp"
#include "search/heuristics/Heuristic.hpp"

// Single-agent A* over full world states. It is primarily used by simpler
// solvers and as a baseline before reservation-aware planning is introduced.
class AStar {
public:
    explicit AStar(const IHeuristic& heuristic)
        : heuristic_(heuristic) {}

    std::vector<Action> search(const Level& level,
                               const State& initial_state,
                               int agent_id);

private:
    const IHeuristic& heuristic_;
};
