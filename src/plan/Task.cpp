#include "plan/Task.hpp"

#include <sstream>

std::string task_type_to_string(TaskType type) {
    switch (type) {
        case TaskType::SolveLevel: return "SolveLevel";
        case TaskType::MoveBoxToGoal: return "MoveBoxToGoal";
        case TaskType::MoveBoxToCell: return "MoveBoxToCell";
        case TaskType::MoveBoxOneStep: return "MoveBoxOneStep";
        case TaskType::MoveAgentTo: return "MoveAgentTo";
        case TaskType::ClearCell: return "ClearCell";
        case TaskType::ClearPath: return "ClearPath";
        case TaskType::PositionAgentForPush: return "PositionAgentForPush";
        case TaskType::PositionAgentForPull: return "PositionAgentForPull";
        case TaskType::Primitive: return "Primitive";
    }
    return "UnknownTask";
}

std::string task_to_string(const HTNTask& task) {
    std::ostringstream out;

    out << task_type_to_string(task.type);

    if (task.agent_id >= 0) {
        out << "(agent=" << task.agent_id;
    } else {
        out << "(";
    }

    if (task.box != ' ') {
        out << ", box=" << task.box;
    }

    if (task.from.row >= 0) {
        out << ", from=(" << task.from.row << "," << task.from.col << ")";
    }

    if (task.to.row >= 0) {
        out << ", to=(" << task.to.row << "," << task.to.col << ")";
    }

    if (task.priority != 0) {
        out << ", priority=" << task.priority;
    }

    out << ")";

    return out.str();
}