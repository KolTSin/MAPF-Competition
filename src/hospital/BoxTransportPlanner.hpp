#pragma once

#include "plan/ReservationTable.hpp"
#include "plan/TaskPlan.hpp"
#include "tasks/Task.hpp"
#include "domain/Level.hpp"
#include "state/State.hpp"
#include "utils/PlanningDeadline.hpp"

// Plans the primitive push/pull sequence that transports one box to a target cell.
class BoxTransportPlanner {
public:
    [[nodiscard]] TaskPlan plan(const Level& level, const State& state, const Task& task) const;
    [[nodiscard]] TaskPlan plan(const Level& level,
                                const State& state,
                                const Task& task,
                                const ReservationTable& reservations,
                                int start_time) const;
    [[nodiscard]] TaskPlan plan(const Level& level,
                                const State& state,
                                const Task& task,
                                const ReservationTable& reservations,
                                int start_time,
                                const PlanningDeadline& deadline) const;
};
