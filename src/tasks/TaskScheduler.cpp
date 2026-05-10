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
// Simple distance heuristic used only for ranking candidate agents.  It ignores
// walls, boxes, and reservations because the real planners below validate the
// path; the scheduler only needs a cheap "try nearby agents first" ordering.
int manhattan(const Position& a, const Position& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

// Return the agents that are allowed to execute a task, ordered by preference.
//
// Input:
//   - level: supplies static agent/box color ownership rules.
//   - state: supplies current simulated positions for distance scoring.
//   - task: the high-level work item that needs an executor.
// Output:
//   - agent ids that may legally do the task.  Agent-only tasks keep their
//     requested agent, while box tasks return all same-colored agents sorted by
//     their Manhattan distance to the current box position.
std::vector<int> candidate_agents_for_task(const Level& level, const State& state, const Task& task) {
    std::vector<std::pair<int, int>> scored;

    // Agent-only tasks are already bound to one agent by the task generator.
    // Returning any other agent would change the meaning of the task.
    if (task.type == TaskType::MoveAgentToGoal || task.type == TaskType::ParkAgentSafely) {
        if (task.agent_id >= 0 && task.agent_id < state.num_agents()) return {task.agent_id};
        return {};
    }

    // Box-moving tasks must be executed by an agent with the same color as the
    // box.  Invalid or missing box identifiers make the task unschedulable.
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

// Some tasks are already satisfied when the scheduler reaches them.  An empty
// primitive action list is only acceptable in those cases; otherwise accepting it
// would mark unfinished work as completed.
bool is_empty_plan_valid(const Task& task, const State& state) {
    if (task.type == TaskType::DeliverBoxToGoal) return task.box_pos == task.goal_pos;
    if (task.type == TaskType::MoveAgentToGoal || task.type == TaskType::ParkAgentSafely) {
        return task.agent_id >= 0 && task.agent_id < state.num_agents() && state.agent_positions[task.agent_id] == task.goal_pos;
    }
    return true;
}
}

std::vector<AgentPlan> TaskScheduler::build_agent_plans(const Level& level, const State& initial_state, const std::vector<Task>& tasks) const {
    // Global reservation state is the contract between independently planned
    // tasks.  Each successful task contributes occupied cells/edges to this
    // table so later planning attempts avoid collisions in space and time.
    ReservationTable reservations;
    std::vector<ScheduledTask> scheduled;

    // The scheduler delegates low-level path construction to specialized
    // planners, and only decides which task/agent/start-time combination to try.
    BoxTransportPlanner box_planner;
    AgentPathPlanner agent_planner;
    DependencyBuilder deps_builder;
    TaskPrioritizer prioritizer;

    // simulated_state is advanced after every accepted task.  It represents the
    // world at the frontier of the partial schedule, so later tasks see the new
    // agent positions and current box locations rather than the original input.
    State simulated_state = initial_state;
    std::vector<Task> mutable_tasks = tasks;
    prioritizer.score(level, simulated_state, mutable_tasks);
    const auto deps = deps_builder.build_graph(mutable_tasks);

    // completed/completion_time track the dependency graph.  agent_available is
    // the earliest timestep at which each agent can accept another task.
    std::unordered_set<int> completed;
    std::unordered_map<int, int> completion_time;
    std::vector<int> agent_available(initial_state.num_agents(), 0);

    // Repeatedly schedule every currently ready task that can be planned.  The
    // loop stops when a full pass cannot add anything, which means all remaining
    // tasks are either blocked by dependencies or currently unplannable.
    bool progress = true;
    while (progress) {
        progress = false;

        // Convert ready task ids back into Task objects, then process higher
        // priority work first.  Dependencies define what is legal; priority
        // defines what is preferred among the legal choices.
        std::vector<Task> ready;
        const auto ready_ids = deps.ready_tasks(completed);
        for (const Task& t : mutable_tasks) {
            if (std::find(ready_ids.begin(), ready_ids.end(), t.task_id) != ready_ids.end()) ready.push_back(t);
        }
        std::sort(ready.begin(), ready.end(), [](const Task& a, const Task& b) { return a.priority > b.priority; });

        // Within one scheduling pass, give each agent at most one newly accepted
        // task.  This prevents a single agent from consuming the entire ready
        // queue before other agents get a chance to plan work in parallel.
        std::unordered_set<int> used_agents;

        for (const Task& task : ready) {
            // A task cannot start before all of its predecessors have ended.
            // Missing predecessors are not ready, so completion_time[pre] is
            // expected to exist for every predecessor seen here.
            int dep_time = 0;
            auto it = deps.predecessors.find(task.task_id);
            if (it != deps.predecessors.end()) {
                for (int pre : it->second) dep_time = std::max(dep_time, completion_time[pre]);
            }

            // Refresh the box position from the simulated world.  Earlier tasks
            // may have moved the same box after task creation, so the planner
            // needs the current input position, not stale task metadata.
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

            // Try legal candidate agents in preference order.  The first
            // successful low-level plan becomes the schedule entry for this
            // high-level task; failed candidates leave the task available for a
            // later pass after other reservations/state may have changed.
            TaskPlan plan;
            int chosen_start = 0;
            bool planned = false;
            const std::vector<int> candidates = candidate_agents_for_task(level, simulated_state, task);
            for (int candidate_agent : candidates) {
                if (candidate_agent < 0 || candidate_agent >= initial_state.num_agents()) continue;
                if (used_agents.count(candidate_agent)) continue;

                chosen_task.agent_id = candidate_agent;
                chosen_start = std::max(agent_available[candidate_agent], dep_time);

                // Agent-only tasks use point-to-point path planning.  Box tasks
                // use transport planning, which plans both agent actions and the
                // box trajectory starting at chosen_start in the global timeline.
                if (chosen_task.type == TaskType::MoveAgentToGoal || chosen_task.type == TaskType::ParkAgentSafely) {
                    std::cerr << "path planning for task: " << chosen_task.task_id << " and assigned agent: " << chosen_task.agent_id << std::endl;
                    plan = agent_planner.plan(level, simulated_state, chosen_task, reservations);
                    std::cerr << "success: " << (plan.success ? "true" : "false") << std::endl;
                } else {
                    std::cerr << "box planning for task: " << chosen_task.task_id << " and assigned agent: " << chosen_task.agent_id << std::endl;
                    reservations.clear();
                    plan = box_planner.plan(level, simulated_state, chosen_task, reservations, chosen_start);
                    std::cerr << "success: " << (plan.success ? "true" : "false") << " reason: " << plan.failure_reason << std::endl;
                }

                if (!plan.success) continue;
                if (plan.agent_plan.actions.empty() && !is_empty_plan_valid(chosen_task, simulated_state)) continue;
                planned = true;
                break;
            }
            if (!planned) continue;

            // Commit the accepted single-task plan to the global schedule.
            // Output of this section is a ScheduledTask plus reservation entries
            // that later tasks must respect.
            const int end_time = chosen_start + static_cast<int>(plan.agent_plan.actions.size());
            scheduled.push_back(ScheduledTask{chosen_task, plan, chosen_start, end_time});
            reservations.reserve_path(plan.agent_plan, chosen_start);
            if (chosen_task.box_id >= 'A' && chosen_task.box_id <= 'Z' && !plan.box_trajectory.empty()) {
                reservations.reserve_box_path(chosen_task.box_id, plan.box_trajectory, chosen_start);
            }

            // Replay the accepted primitive actions into simulated_state.  This
            // updates the scheduler's working copy of agent and box positions so
            // the next task receives intuitive inputs: the world after all work
            // accepted so far has completed.
            Position cur = simulated_state.agent_positions[chosen_task.agent_id];
            for (const Action& a : plan.agent_plan.actions) {
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

            // Mark the task as finished in both the dependency graph and the
            // per-agent timeline.  These values become inputs to future start
            // time calculations.
            completed.insert(chosen_task.task_id);
            completion_time[chosen_task.task_id] = end_time;
            agent_available[chosen_task.agent_id] = end_time;
            used_agents.insert(chosen_task.agent_id);
            progress = true;
        }
    }

    // Transform sparse scheduled tasks into dense per-agent timelines.  Missing
    // time slots become NoOp actions so all agent plans share a common global
    // clock before they are merged into the final joint Plan output.
    std::vector<AgentPlan> agent_plans(initial_state.num_agents());
    if (scheduled.empty()) return {};
    for (int agent = 0; agent < initial_state.num_agents(); ++agent) {
        agent_plans[agent].agent = agent;
    }
    for (const auto& st : scheduled) {
        auto& timeline = agent_plans[st.task.agent_id].actions;
        if (static_cast<int>(timeline.size()) < st.start_time) timeline.resize(st.start_time, Action::noop());
        if (static_cast<int>(timeline.size()) < st.end_time) timeline.resize(st.end_time, Action::noop());
        for (int i = 0; i < static_cast<int>(st.plan.agent_plan.actions.size()); ++i) {
            timeline[st.start_time + i] = st.plan.agent_plan.actions[static_cast<std::size_t>(i)];
        }
    }

    // Reconstruct positions from actions for each agent because PlanMerger and
    // downstream validators consume both the action sequence and the derived
    // trajectory.  The input is an action timeline; the output is positions[0]
    // at the initial state plus one position after each action.
    for (int agent = 0; agent < initial_state.num_agents(); ++agent) {
        AgentPlan& plan = agent_plans[agent];
        Position cur = initial_state.agent_positions[agent];
        plan.positions.clear();
        plan.positions.push_back(cur);
        for (const Action& action : plan.actions) {
            cur = ActionSemantics::compute_effect(cur, action).agent_to;
            plan.positions.push_back(cur);
        }
    }

    return agent_plans;
}

Plan TaskScheduler::build_plan(const Level& level, const State& initial_state, const std::vector<Task>& tasks) const {
    const std::vector<AgentPlan> agent_plans = build_agent_plans(level, initial_state, tasks);
    if (agent_plans.empty()) return {};
    return PlanMerger::merge_agent_plans(agent_plans, initial_state.num_agents());
}

