#include "tasks/TaskScheduler.hpp"

#include "hospital/AgentPathPlanner.hpp"
#include "hospital/BoxTransportPlanner.hpp"
#include "plan/ConflictDetector.hpp"
#include "plan/PlanMerger.hpp"
#include "plan/ReservationTable.hpp"

Plan TaskScheduler::build_plan(const Level& level, const State& initial_state, const std::vector<Task>& tasks) const {
    std::vector<std::vector<Action>> agent_plans(initial_state.num_agents());
    ReservationTable reservations;

    BoxTransportPlanner box_planner;
    AgentPathPlanner agent_planner;

    for (const Task& task : tasks) {
        TaskPlan plan;
        if (task.type == TaskType::MoveAgentToGoal) {
            plan = agent_planner.plan(level, initial_state, task, reservations);
        } else {
            plan = box_planner.plan(level, initial_state, task);
        }
        if (!plan.success) {
            continue;
        }
        agent_plans[task.agent_id].insert(agent_plans[task.agent_id].end(), plan.primitive_actions.begin(), plan.primitive_actions.end());
        reservations.reserve_path(plan.primitive_actions, initial_state.agent_positions[task.agent_id], task.agent_id);
    }

    Plan merged = PlanMerger::merge_agent_plans(agent_plans, initial_state.num_agents());
    if (ConflictDetector::has_conflict(merged, initial_state, nullptr)) {
        return Plan{};
    }
    return merged;
}
