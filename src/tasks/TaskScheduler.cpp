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
#include <unordered_set>

Plan TaskScheduler::build_plan(const Level& level, const State& initial_state, const std::vector<Task>& tasks) const {
    std::vector<std::vector<Action>> agent_plans(initial_state.num_agents());
    ReservationTable reservations;
    std::vector<ScheduledTask> scheduled;

    BoxTransportPlanner box_planner;
    AgentPathPlanner agent_planner;
    DependencyBuilder deps_builder;
    TaskPrioritizer prioritizer;

    State simulated_state = initial_state;
    std::vector<Task> mutable_tasks = tasks;
    prioritizer.score(level, simulated_state, mutable_tasks);
    const auto deps = deps_builder.build(mutable_tasks);
    std::unordered_set<int> completed;

    bool progress = true;
    int global_time = 0;
    while (progress) {
        progress = false;
        std::vector<Task> ready;
        for (const Task& t : mutable_tasks) {
            if (completed.count(t.task_id)) continue;
            bool ok = true;
            auto it = deps.find(t.task_id);
            if (it != deps.end()) {
                for (int pre : it->second) if (!completed.count(pre)) { ok = false; break; }
            }
            if (ok) ready.push_back(t);
        }
        std::sort(ready.begin(), ready.end(), [](const Task& a, const Task& b) { return a.priority > b.priority; });

        for (const Task& task : ready) {
            TaskPlan plan;
            if (task.type == TaskType::MoveAgentToGoal) plan = agent_planner.plan(level, simulated_state, task, reservations);
            else plan = box_planner.plan(level, simulated_state, task);
            if (!plan.success) continue;

            scheduled.push_back(ScheduledTask{task, plan, global_time, global_time + static_cast<int>(plan.primitive_actions.size())});
            agent_plans[task.agent_id].insert(agent_plans[task.agent_id].end(), plan.primitive_actions.begin(), plan.primitive_actions.end());
            reservations.reserve_path(plan.primitive_actions, simulated_state.agent_positions[task.agent_id], task.agent_id, global_time);

            Position cur = simulated_state.agent_positions[task.agent_id];
            for (const Action& a : plan.primitive_actions) {
                cur = ActionSemantics::compute_effect(cur, a).agent_to;
            }
            simulated_state.agent_positions[task.agent_id] = cur;

            completed.insert(task.task_id);
            global_time += static_cast<int>(plan.primitive_actions.size());
            progress = true;
            break;
        }
    }

    Plan merged = PlanMerger::merge_agent_plans(agent_plans, initial_state.num_agents());
    if (ConflictDetector::has_conflict(merged, initial_state, nullptr)) return Plan{};
    return merged;
}
