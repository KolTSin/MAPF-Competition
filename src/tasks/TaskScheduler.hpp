#pragma once

#include "domain/Level.hpp"
#include "plan/Plan.hpp"
#include "state/State.hpp"
#include "tasks/Task.hpp"

#include <vector>

class TaskScheduler {
public:
    [[nodiscard]] Plan build_plan(const Level& level, const State& initial_state, const std::vector<Task>& tasks) const;
};
