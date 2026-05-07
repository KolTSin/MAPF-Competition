#pragma once

#include "actions/Action.hpp"
#include "domain/Level.hpp"
#include "domain/Position.hpp"
#include "plan/ReservationTable.hpp"
#include "plan/AgentPlan.hpp"
#include "search/heuristics/Heuristic.hpp"

#include <vector>

// Lightweight position/time node kept for compatibility with earlier planners.
struct SpaceTimeNode {
    Position pos{};
    int time{0};

    int g{0};
    int h{0};

    int parent_index{-1};
    Action action{Action::noop()};

    [[nodiscard]] int f() const noexcept {
        return g + h;
    }
};

// A* variant that includes timestep in the closed-list key and rejects moves
// that conflict with reservations made by previously scheduled agents/boxes.
class SpaceTimeAStar {
public:
    explicit SpaceTimeAStar(const IHeuristic& heuristic)
        : heuristic_(heuristic) {}

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
