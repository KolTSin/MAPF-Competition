#pragma once

#include "actions/Action.hpp"
#include "domain/Level.hpp"
#include "domain/Position.hpp"
#include "plan/ReservationTable.hpp"
#include "search/heuristics/Heuristic.hpp"

#include <vector>

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

class SpaceTimeAStar {
public:
    explicit SpaceTimeAStar(const IHeuristic& heuristic):
        heuristic_(heuristic)
        {
        }

    std::vector<Action> search(
        const Level& level,
        const State& initial_state,
        const int agent,
        const Position& goal_pos,
        const ReservationTable& reservations
    );
private:
    const IHeuristic& heuristic_;
};