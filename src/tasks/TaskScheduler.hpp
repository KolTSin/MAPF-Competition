#pragma once

#include "domain/Level.hpp"
#include "plan/Plan.hpp"
#include "state/State.hpp"
#include "tasks/Task.hpp"
#include "plan/TaskPlan.hpp"

#include <vector>

// A single high-level task after it has been placed on the global timeline.
//
// `task` keeps the domain-level intent (move this agent, deliver this box).
// `plan` is the low-level output produced by an agent or box planner.
// `start_time` and `end_time` are absolute timesteps in the final joint plan,
// so downstream code can insert noops before/after this fragment as needed.
struct ScheduledTask {
    Task task;
    TaskPlan plan;
    int start_time{0};
    int end_time{0};
};

// Converts high-level tasks into one executable joint plan.
//
// Input:
// - `level`: static world information such as walls, goals, and colors.
// - `initial_state`: dynamic world information at timestep 0.
// - `tasks`: unordered high-level work items produced by task decomposition.
//
// Output:
// - A `Plan` containing synchronized primitive actions for all agents.
// - An empty plan when no task can be scheduled safely.
//
// The scheduler is intentionally an orchestrator: it scores tasks, respects
// dependencies, chooses compatible agents, asks specialized planners for
// low-level actions, reserves accepted paths, and finally merges per-agent
// timelines into the joint plan consumed by solvers.
class TaskScheduler {
public:
    [[nodiscard]] Plan build_plan(const Level& level, const State& initial_state, const std::vector<Task>& tasks) const;
};
