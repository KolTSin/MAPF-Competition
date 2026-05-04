#pragma once

#include "plan/TaskPlan.hpp"
#include "tasks/Task.hpp"
#include "domain/Level.hpp"
#include "state/State.hpp"

class BoxTransportPlanner {
public:
    [[nodiscard]] TaskPlan plan(const Level& level, const State& state, const Task& task) const;
};
