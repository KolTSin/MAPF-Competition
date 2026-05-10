#pragma once

#include "domain/Level.hpp"
#include "plan/ReservationTable.hpp"
#include "plan/TaskPlan.hpp"
#include "state/State.hpp"
#include "tasks/Task.hpp"

// Plans agent-only movement for tasks that do not require moving a box.
class AgentPathPlanner {
public:
    [[nodiscard]] TaskPlan plan(const Level& level, const State& state, const Task& task, const ReservationTable& reservations) const;
    [[nodiscard]] TaskPlan plan(const Level& level, const State& state, const Task& task, const ReservationTable& reservations, int start_time) const;
};
