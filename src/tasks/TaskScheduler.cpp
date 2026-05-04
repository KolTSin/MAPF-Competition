#include "tasks/TaskScheduler.hpp"

#include "actions/ActionSemantics.hpp"
#include "hospital/AgentPathPlanner.hpp"
#include "hospital/BoxTransportPlanner.hpp"
#include "plan/ConflictDetector.hpp"
#include "plan/PlanMerger.hpp"
#include "plan/ReservationTable.hpp"
#include "tasks/DependencyBuilder.hpp"
#include "tasks/TaskPrioritizer.hpp"

#include <algorithm>
#include <cassert>
#include <unordered_map>
#include <unordered_set>

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
        for (const Task& t : mutable_tasks) if (std::find(ready_ids.begin(), ready_ids.end(), t.task_id) != ready_ids.end()) ready.push_back(t);
        std::sort(ready.begin(), ready.end(), [](const Task& a, const Task& b) { return a.priority > b.priority; });

        std::vector<Task> wave = ready;
        std::sort(wave.begin(), wave.end(), [](const Task& a, const Task& b) { return a.priority > b.priority; });
        std::unordered_set<int> used_agents;

        for (const Task& task : wave) {
            if (task.agent_id < 0 || task.agent_id >= initial_state.num_agents()) continue;
            if (used_agents.count(task.agent_id)) continue;

            int dep_time = 0;
            auto it = deps.predecessors.find(task.task_id);
            if (it != deps.predecessors.end()) {
                for (int pre : it->second) dep_time = std::max(dep_time, completion_time[pre]);
            }
            const int start_time = std::max(agent_available[task.agent_id], dep_time);

            TaskPlan plan;
            if (task.type == TaskType::MoveAgentToGoal) plan = agent_planner.plan(level, simulated_state, task, reservations);
            else plan = box_planner.plan(level, simulated_state, task);
            if (!plan.success) continue;

            const int end_time = start_time + static_cast<int>(plan.primitive_actions.size());
            scheduled.push_back(ScheduledTask{task, plan, start_time, end_time});
            reservations.reserve_path(plan.primitive_actions, simulated_state.agent_positions[task.agent_id], task.agent_id, start_time);

            Position cur = simulated_state.agent_positions[task.agent_id];
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
            simulated_state.agent_positions[task.agent_id] = cur;

            completed.insert(task.task_id);
            completion_time[task.task_id] = end_time;
            agent_available[task.agent_id] = end_time;
            used_agents.insert(task.agent_id);
            progress = true;
        }
    }

    std::vector<std::vector<Action>> agent_plans(initial_state.num_agents());
    for (const auto& st : scheduled) {
        auto& timeline = agent_plans[st.task.agent_id];
        if (static_cast<int>(timeline.size()) < st.start_time) {
            timeline.resize(st.start_time, Action::noop());
        }
        if (static_cast<int>(timeline.size()) < st.end_time) {
            timeline.resize(st.end_time, Action::noop());
        }
        for (int i = 0; i < static_cast<int>(st.plan.primitive_actions.size()); ++i) {
            timeline[st.start_time + i] = st.plan.primitive_actions[static_cast<std::size_t>(i)];
        }
    }
    for (const auto& st : scheduled) {
        const auto& timeline = agent_plans[st.task.agent_id];
        for (int t = 0; t < st.start_time; ++t) {
            assert(timeline[static_cast<std::size_t>(t)].type == ActionType::NoOp);
        }
    }

    Plan merged = PlanMerger::merge_agent_plans(agent_plans, initial_state.num_agents());
    if (ConflictDetector::has_conflict(merged, initial_state, nullptr)) return Plan{};
    return merged;
}
