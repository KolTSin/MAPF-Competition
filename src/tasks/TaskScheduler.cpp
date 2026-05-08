#include "tasks/TaskScheduler.hpp"

#include "actions/ActionSemantics.hpp"
#include "hospital/AgentPathPlanner.hpp"
#include "hospital/BoxTransportPlanner.hpp"
#include "plan/PlanMerger.hpp"
#include "plan/ReservationTable.hpp"
#include "tasks/DependencyBuilder.hpp"
#include "tasks/TaskPrioritizer.hpp"

#include <algorithm>
#include <cassert>
#include <unordered_map>
#include <unordered_set>
#include <iostream>

namespace {
// Distance heuristic used only for ranking candidate agents.  It ignores walls,
// boxes, and reservations because the actual planners perform the expensive
// feasibility checks later; here we just want a stable, intuitive ordering
// where nearby agents try the task before farther agents.
int manhattan(const Position& a, const Position& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

// Return agent ids that are allowed to execute `task`, ordered by preference.
//
// Input:
// - `level` provides colors, which encode which agents may move which boxes.
// - `state` provides current agent locations for the proximity ranking.
// - `task` describes either an agent-only goal or a box-delivery goal.
//
// Output:
// - For agent-only tasks, the only valid agent is the task's assigned agent.
// - For box tasks, every color-compatible agent is returned, nearest first.
// - Invalid or underspecified tasks return an empty list so the scheduler can
//   simply skip them instead of special-casing errors in the main loop.
std::vector<int> candidate_agents_for_task(const Level& level, const State& state, const Task& task) {
    std::vector<std::pair<int, int>> scored;
    if (task.type == TaskType::MoveAgentToGoal || task.type == TaskType::ParkAgentSafely) {
        if (task.agent_id >= 0 && task.agent_id < state.num_agents()) return {task.agent_id};
        return {};
    }
    if (task.box_id < 'A' || task.box_id > 'Z') return {};
    const Color box_color = level.box_colors[static_cast<std::size_t>(task.box_id - 'A')];
    for (int a = 0; a < state.num_agents(); ++a) {
        if (level.agent_colors[static_cast<std::size_t>(a)] != box_color) continue;
        scored.emplace_back(manhattan(state.agent_positions[static_cast<std::size_t>(a)], task.box_pos), a);
    }
    std::sort(scored.begin(), scored.end());
    std::vector<int> out;
    for (const auto& [_, a] : scored) out.push_back(a);
    return out;
}

// Some planners can legitimately return a successful plan with no primitive
// actions: the requested object is already at its target.  This helper checks
// that an empty output really means "already solved" rather than "planner gave
// up but marked success".
bool is_empty_plan_valid(const Task& task, const State& state) {
    if (task.type == TaskType::DeliverBoxToGoal) return task.box_pos == task.goal_pos;
    if (task.type == TaskType::MoveAgentToGoal || task.type == TaskType::ParkAgentSafely) {
        return task.agent_id >= 0 && task.agent_id < state.num_agents() && state.agent_positions[task.agent_id] == task.goal_pos;
    }
    return true;
}
}

Plan TaskScheduler::build_plan(const Level& level, const State& initial_state, const std::vector<Task>& tasks) const {
    // Global output being built:
    // - `reservations` answers "which cells/edges/boxes are occupied at time t?"
    //   so each newly planned task avoids work that has already been accepted.
    // - `scheduled` stores successful single-task plans together with their
    //   absolute start/end times; it is converted to a joint Plan at the end.
    ReservationTable reservations;
    std::vector<ScheduledTask> scheduled;

    // Collaborators keep the scheduler focused on orchestration:
    // planners produce primitive actions, the dependency builder decides which
    // tasks are unblocked, and the prioritizer chooses the order among tasks
    // that are simultaneously ready.
    BoxTransportPlanner box_planner;
    AgentPathPlanner agent_planner;
    DependencyBuilder deps_builder;
    TaskPrioritizer prioritizer;

    // `simulated_state` is advanced after every accepted task.  Subsequent
    // tasks therefore see the boxes and agents where the scheduler expects them
    // to be, not where they were in the original input state.  The input `tasks`
    // are copied so priorities can be written without mutating the caller's data.
    State simulated_state = initial_state;
    std::vector<Task> mutable_tasks = tasks;
    // Priority and dependency setup:
    // - `priority` is a soft preference used when several tasks are legal.
    // - dependencies are hard ordering constraints; a task is invisible to the
    //   planning loop until all of its predecessors are completed.
    prioritizer.score(level, simulated_state, mutable_tasks);
    const auto deps = deps_builder.build_graph(mutable_tasks);

    // Per-task and per-agent clocks.  A task may start only after its dependency
    // completion time and after its chosen agent becomes available.
    std::unordered_set<int> completed;
    std::unordered_map<int, int> completion_time;
    std::vector<int> agent_available(initial_state.num_agents(), 0);

    // Greedily schedule waves of currently ready tasks.  Each pass attempts at
    // most one task per agent (`used_agents`) so independent tasks can share the
    // same wave while still avoiding double-booking an agent.  The loop stops
    // when a full pass cannot add anything, which means remaining tasks are
    // blocked by dependencies or no collision-free plan was found for them.
    bool progress = true;
    while (progress) {
        progress = false;
        // Translate ready task ids back into Task records, then prefer the
        // highest priority tasks.  The output of this block is an ordered list
        // of high-level intentions to try in this wave.
        std::vector<Task> ready;
        const auto ready_ids = deps.ready_tasks(completed);
        for (const Task& t : mutable_tasks) {
            if (std::find(ready_ids.begin(), ready_ids.end(), t.task_id) != ready_ids.end()) ready.push_back(t);
        }
        std::sort(ready.begin(), ready.end(), [](const Task& a, const Task& b) { return a.priority > b.priority; });
        std::unordered_set<int> used_agents;

        for (const Task& task : ready) {
            // Earliest time allowed by dependencies.  If a task depends on
            // multiple predecessors, it must wait for the latest one to finish.
            int dep_time = 0;
            auto it = deps.predecessors.find(task.task_id);
            if (it != deps.predecessors.end()) {
                for (int pre : it->second) dep_time = std::max(dep_time, completion_time[pre]);
            }

            // Refresh movable object positions from the simulated world.
            // Earlier tasks may have transported a box, so the original
            // `task.box_pos` can be stale by the time this task becomes ready.
            Task chosen_task = task;
            if (chosen_task.box_id >= 'A' && chosen_task.box_id <= 'Z') {
                for (int br = 0; br < simulated_state.rows; ++br) {
                    for (int bc = 0; bc < simulated_state.cols; ++bc) {
                        if (simulated_state.box_at(br, bc) == chosen_task.box_id) {
                            chosen_task.box_pos = Position{br, bc};
                            br = simulated_state.rows;
                            break;
                        }
                    }
                }
            }
            // Try each valid agent until one produces a collision-free plan.
            // Input to the planner is the current simulated state, the selected
            // candidate agent, the global reservation table, and the earliest
            // start time for box transport.  Output is a TaskPlan containing
            // primitive actions plus optional box trajectory metadata.
            TaskPlan plan;
            int chosen_start = 0;
            bool planned = false;
            const std::vector<int> candidates = candidate_agents_for_task(level, simulated_state, task);
            for (int candidate_agent : candidates) {
                if (candidate_agent < 0 || candidate_agent >= initial_state.num_agents()) continue;
                if (used_agents.count(candidate_agent)) continue;
                chosen_task.agent_id = candidate_agent;
                chosen_start = std::max(agent_available[candidate_agent], dep_time);
                if (chosen_task.type == TaskType::MoveAgentToGoal || chosen_task.type == TaskType::ParkAgentSafely) {
                    plan = agent_planner.plan(level, simulated_state, chosen_task, reservations);
                } else {
                    plan = box_planner.plan(level, simulated_state, chosen_task, reservations, chosen_start);
                }
                if (!plan.success) continue;
                // Empty action lists are accepted only when the goal is already
                // true in `simulated_state`; otherwise the scheduler would
                // falsely mark unsolved work as complete.
                if (plan.primitive_actions.empty() && !is_empty_plan_valid(chosen_task, simulated_state)) continue;
                planned = true;
                break;
            }
            if (!planned) continue;

            // Commit the accepted plan to the global timeline.  From this point
            // onward, later tasks must route around the reserved agent path and,
            // for box tasks, around the transported box trajectory as well.
            const int end_time = chosen_start + static_cast<int>(plan.primitive_actions.size());
            scheduled.push_back(ScheduledTask{chosen_task, plan, chosen_start, end_time});
            reservations.reserve_path(plan.primitive_actions, simulated_state.agent_positions[chosen_task.agent_id], chosen_task.agent_id, chosen_start);
            if (chosen_task.box_id >= 'A' && chosen_task.box_id <= 'Z' && !plan.box_trajectory.empty()) {
                reservations.reserve_box_path(chosen_task.box_id, plan.box_trajectory, chosen_start);
            }

            // Replay the primitive actions into `simulated_state`.  This is the
            // scheduler's state transition: input is the state before the task,
            // output is the predicted state after the task finishes.
            Position cur = simulated_state.agent_positions[chosen_task.agent_id];
            for (const Action& a : plan.primitive_actions) {
                const ActionEffect eff = ActionSemantics::compute_effect(cur, a);
                cur = eff.agent_to;
                if (eff.moves_box && simulated_state.in_bounds(eff.box_from.row, eff.box_from.col) && simulated_state.in_bounds(eff.box_to.row, eff.box_to.col)) {
                    const char moved = simulated_state.box_at(eff.box_from.row, eff.box_from.col);
                    if (moved != '\0') {
                        simulated_state.set_box(eff.box_from.row, eff.box_from.col, '\0');
                        simulated_state.set_box(eff.box_to.row, eff.box_to.col, moved);
                    }
                }
            }
            simulated_state.agent_positions[chosen_task.agent_id] = cur;

            // Record completion so dependency checks and agent availability for
            // future waves reflect this newly scheduled work.
            completed.insert(chosen_task.task_id);
            completion_time[chosen_task.task_id] = end_time;
            agent_available[chosen_task.agent_id] = end_time;
            used_agents.insert(chosen_task.agent_id);
            progress = true;
        }
    }

    // Convert scheduled single-agent task fragments into the solver's joint
    // Plan format.  Each agent receives its own timeline with noop padding until
    // the task's absolute start time; PlanMerger then zips those timelines into
    // one action vector per timestep.
    std::vector<std::vector<Action>> agent_plans(initial_state.num_agents());
    if (scheduled.empty()) return {};
    for (const auto& st : scheduled) {
        auto& timeline = agent_plans[st.task.agent_id];
        if (static_cast<int>(timeline.size()) < st.start_time) timeline.resize(st.start_time, Action::noop());
        if (static_cast<int>(timeline.size()) < st.end_time) timeline.resize(st.end_time, Action::noop());
        for (int i = 0; i < static_cast<int>(st.plan.primitive_actions.size()); ++i) {
            timeline[st.start_time + i] = st.plan.primitive_actions[static_cast<std::size_t>(i)];
        }
    }
    return PlanMerger::merge_agent_plans(agent_plans, initial_state.num_agents());
}