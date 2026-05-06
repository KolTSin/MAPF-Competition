#pragma once

#include "plan/ReservationTable.hpp"
#include "plan/TaskPlan.hpp"
#include "tasks/Task.hpp"
#include "domain/Level.hpp"
#include "state/State.hpp"

class BoxTransportPlanner {
public:
    [[nodiscard]] TaskPlan plan(const Level& level, const State& state, const Task& task) const;
    [[nodiscard]] TaskPlan plan(const Level& level,
                                const State& state,
                                const Task& task,
                                const ReservationTable& reservations,
                                int start_time) const;
};
