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
int manhattan(const Position& a, const Position& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

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

bool is_empty_plan_valid(const Task& task, const State& state) {
    if (task.type == TaskType::DeliverBoxToGoal) return task.box_pos == task.goal_pos;
    if (task.type == TaskType::MoveAgentToGoal || task.type == TaskType::ParkAgentSafely) {
        return task.agent_id >= 0 && task.agent_id < state.num_agents() && state.agent_positions[task.agent_id] == task.goal_pos;
    }
    return true;
}
}

Plan TaskScheduler::build_plan(const Level& level, const State& initial_state, const std::vector<Task>& tasks) const {
    ReservationTable reservations;
    std::vector<ScheduledTask> scheduled;

    BoxTransportPlanner box_planner;
    AgentPathPlanner agent_planner;
    DependencyBuilder deps_builder;
    TaskPrioritizer prioritizer;

    State simulated_state = initial_state;
    std::vector<Task> mutable_tasks = tasks;
    prioritizer.score(level, simulated_state, mutable_tasks);
    const auto deps = deps_builder.build_graph(mutable_tasks);
    std::unordered_set<int> completed;
    std::unordered_map<int, int> completion_time;
    std::vector<int> agent_available(initial_state.num_agents(), 0);

    bool progress = true;
    while (progress) {
        progress = false;
        std::vector<Task> ready;
        const auto ready_ids = deps.ready_tasks(completed);
        for (const Task& t : mutable_tasks) {
            if (std::find(ready_ids.begin(), ready_ids.end(), t.task_id) != ready_ids.end()) ready.push_back(t);
        }
        std::sort(ready.begin(), ready.end(), [](const Task& a, const Task& b) { return a.priority > b.priority; });
        std::unordered_set<int> used_agents;

        for (const Task& task : ready) {
            int dep_time = 0;
            auto it = deps.predecessors.find(task.task_id);
            if (it != deps.predecessors.end()) {
                for (int pre : it->second) dep_time = std::max(dep_time, completion_time[pre]);
            }

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
                if (plan.agent_plan.actions.empty() && !is_empty_plan_valid(chosen_task, simulated_state)) continue;
                planned = true;
                break;
            }
            if (!planned) continue;

            const int end_time = chosen_start + static_cast<int>(plan.agent_plan.actions.size());
            scheduled.push_back(ScheduledTask{chosen_task, plan, chosen_start, end_time});
            reservations.reserve_path(plan.agent_plan, chosen_start);
            if (chosen_task.box_id >= 'A' && chosen_task.box_id <= 'Z' && !plan.box_trajectory.empty()) {
                reservations.reserve_box_path(chosen_task.box_id, plan.box_trajectory, chosen_start);
            }

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

            completed.insert(chosen_task.task_id);
            completion_time[chosen_task.task_id] = end_time;
            agent_available[chosen_task.agent_id] = end_time;
            used_agents.insert(chosen_task.agent_id);
            progress = true;
        }
    }

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
    return PlanMerger::merge_agent_plans(agent_plans, initial_state.num_agents());
}