#include "tasks/Task.hpp"

#include <sstream>

std::string Task::describe() const {
    std::ostringstream out;
    out << "Task#" << task_id << " ";

    switch (type) {
        case TaskType::DeliverBoxToGoal:
            out << "DeliverBoxToGoal box=" << box_id
                << " goal=" << goal_symbol
                << " agent=" << agent_id;
            break;
        case TaskType::MoveBlockingBoxToParking:
            out << "MoveBlockingBoxToParking box=" << box_id
                << " parking=(" << parking_pos.row << "," << parking_pos.col << ")"
                << " agent=" << agent_id;
            break;
        case TaskType::MoveAgentToGoal:
            out << "MoveAgentToGoal agent=" << agent_id
                << " target_goal=" << goal_symbol;
            break;
        case TaskType::ParkAgentSafely:
            out << "ParkAgentSafely agent=" << agent_id
                << " parking=(" << parking_pos.row << "," << parking_pos.col << ")";
            break;
    }

    out << " prio=" << priority;
    if (!dependencies.empty()) {
        out << " deps=";
        for (int i : dependencies) out << ", " << i;
    }

    return out.str();
}
