#include "tasks/TaskPrioritizer.hpp"

#include <algorithm>

namespace {
int manhattan(const Position& a, const Position& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

bool is_box_goal(char goal) {
    return goal >= 'A' && goal <= 'Z';
}

int corridor_access_bonus(const Level& level, const State& state, const Task& task) {
    if (task.type != TaskType::DeliverBoxToGoal) return 0;
    if (task.agent_id < 0 || task.agent_id >= state.num_agents()) return 0;

    const Position agent = state.agent_positions[static_cast<std::size_t>(task.agent_id)];
    if (agent.row != task.goal_pos.row || agent.col == task.goal_pos.col) return 0;

    const int step = (task.goal_pos.col > agent.col) ? 1 : -1;
    int blocking_goals = 0;
    for (int c = agent.col + step; c != task.goal_pos.col; c += step) {
        if (!level.in_bounds(agent.row, c) || level.is_wall(agent.row, c)) break;
        const char goal = level.goal_at(agent.row, c);
        if (!is_box_goal(goal) || goal == task.box_id) continue;
        if (state.box_at(agent.row, c) == goal) continue;
        ++blocking_goals;
    }

    // Each unsatisfied box goal between the assigned agent and this task's goal
    // is likely to become a permanent obstacle in a narrow corridor.  Give the
    // far-side task enough lift to run before those nearer deliveries seal the
    // corridor, without overwhelming the existing distance-based ordering.
    return blocking_goals * 3;
}
}

void TaskPrioritizer::score(const Level& level, const State& state, std::vector<Task>& tasks) const {
    for (Task& task : tasks) {
        int urgency = 0;
        int unlock = 0;
        int chokepoint_risk = 0;
        int transport_cost = 0;

        if (task.type == TaskType::DeliverBoxToGoal) {
            urgency = std::max(0, 30 - manhattan(task.box_pos, task.goal_pos));
            unlock = 20 + corridor_access_bonus(level, state, task);
            transport_cost = manhattan(state.agent_positions[task.agent_id], task.box_pos) + manhattan(task.box_pos, task.goal_pos);
        } else if (task.type == TaskType::MoveBlockingBoxToParking) {
            urgency = 25;
            unlock = 30;
            chokepoint_risk = 10;
            transport_cost = manhattan(task.box_pos, task.parking_pos);
        } else {
            urgency = 10;
            transport_cost = manhattan(state.agent_positions[task.agent_id], task.goal_pos);
        }

        task.priority = urgency + unlock - chokepoint_risk - transport_cost;
    }
}
