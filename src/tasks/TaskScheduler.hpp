#pragma once

#include "domain/Level.hpp"
#include "plan/Plan.hpp"
#include "state/State.hpp"
#include "tasks/Task.hpp"
#include "plan/TaskPlan.hpp"

#include <vector>

// A single high-level task after it has been accepted into the global schedule.
//
// Input carried into this record:
//   - task: the declarative work item, including the chosen agent and any box or
//     goal positions used by the low-level planner.
//   - plan: the primitive agent actions and optional box trajectory generated
//     for exactly this task.
//   - start_time/end_time: the half-open time interval [start_time, end_time)
//     occupied on the shared timeline.
// Output/use:
//   - TaskScheduler later expands these sparse task intervals into one dense
//     per-agent action sequence padded with NoOps where the agent is idle.
struct ScheduledTask {
    Task task;
    TaskPlan plan;
    int start_time{0};
    int end_time{0};
};

// Converts high-level tasks into one executable joint plan.
//
// The scheduler's job is orchestration rather than pathfinding:
//   1. score and dependency-order tasks,
//   2. choose a legal agent and start time for each ready task,
//   3. ask the specialized low-level planner for primitive actions,
//   4. reserve the resulting time/space trajectory so later tasks avoid it, and
//   5. merge all accepted single-task plans into the final Plan output.
//
// Inputs are immutable level geometry, the initial multi-agent state, and a list
// of declarative Task objects.  The output is a joint Plan whose rows are the
// timestep-by-timestep actions for all agents.
class TaskScheduler {
public:
    [[nodiscard]] Plan build_plan(const Level& level, const State& initial_state, const std::vector<Task>& tasks) const;
};
