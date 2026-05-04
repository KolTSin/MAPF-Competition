#pragma once

#include "domain/Level.hpp"
#include "plan/TaskPlan.hpp"
#include "state/State.hpp"
#include "tasks/Task.hpp"

class LocalRepair {
public:
    [[nodiscard]] TaskPlan repair(const Level& level, const State& state, const Task& task, const TaskPlan& failed) const;
};
