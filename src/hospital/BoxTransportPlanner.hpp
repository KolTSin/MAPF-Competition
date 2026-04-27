#pragma once

#include "actions/Action.hpp"
#include "domain/Level.hpp"
#include "domain/Position.hpp"
#include "hospital/HospitalTask.hpp"
#include "hospital/MapAnalysis.hpp"
#include "state/State.hpp"

#include <optional>
#include <vector>

class BoxTransportPlanner {
public:
    explicit BoxTransportPlanner(const MapAnalysis& analysis);

    std::vector<Action> plan_for_task(const Level& level, State& state, const HospitalTask& task) const;

private:
    const MapAnalysis& analysis_;

    [[nodiscard]] static std::optional<Direction> direction_from_delta(int dr, int dc);
    [[nodiscard]] static Position add(const Position& p, Direction dir);
    [[nodiscard]] static int manhattan(const Position& a, const Position& b) noexcept;

    [[nodiscard]] static std::optional<Position> find_box(const State& state, char box_symbol);
    [[nodiscard]] std::vector<Position> shortest_box_path(
        const Level& level,
        const State& state,
        int agent,
        const Position& agent_start,
        const Position& box_start,
        const Position& box_goal,
        char box_symbol) const;
    [[nodiscard]] std::vector<Action> shortest_agent_walk(const Level& level, const State& state, int agent, const Position& from, const Position& to) const;

    static void apply_actions(const Level& level, State& state, int agent, const std::vector<Action>& actions);
};
