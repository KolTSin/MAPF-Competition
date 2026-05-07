#pragma once

#include "domain/Level.hpp"
#include "plan/Plan.hpp"
#include "state/State.hpp"
#include "tasks/Task.hpp"
#include "plan/TaskPlan.hpp"

#include <vector>

// A planned task annotated with when it occupies the global timeline.
struct ScheduledTask {
    Task task;
    TaskPlan plan;
    int start_time{0};
    int end_time{0};
};

// Converts high-level tasks into one executable joint plan, using reservations
// to keep concurrently planned work from colliding.
class TaskScheduler {
public:
    [[nodiscard]] Plan build_plan(const Level& level, const State& initial_state, const std::vector<Task>& tasks) const;
};
