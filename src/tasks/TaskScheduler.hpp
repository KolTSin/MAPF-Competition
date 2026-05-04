#pragma once

#include "domain/Level.hpp"
#include "plan/Plan.hpp"
#include "state/State.hpp"
#include "tasks/Task.hpp"
#include "plan/TaskPlan.hpp"

#include <vector>

struct ScheduledTask {
    Task task;
    TaskPlan plan;
    int start_time{0};
    int end_time{0};
};

class TaskScheduler {
public:
    [[nodiscard]] Plan build_plan(const Level& level, const State& initial_state, const std::vector<Task>& tasks) const;
};
