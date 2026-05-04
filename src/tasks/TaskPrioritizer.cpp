#include "tasks/TaskPrioritizer.hpp"

#include <algorithm>

namespace {
int manhattan(const Position& a, const Position& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}
}

void TaskPrioritizer::score(const Level& level, const State& state, std::vector<Task>& tasks) const {
    (void)level;
    for (Task& task : tasks) {
        int urgency = 0;
        int unlock = 0;
        int chokepoint_risk = 0;
        int transport_cost = 0;

        if (task.type == TaskType::DeliverBoxToGoal) {
            urgency = std::max(0, 30 - manhattan(task.box_pos, task.goal_pos));
            unlock = 20;
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
