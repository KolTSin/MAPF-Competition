#pragma once

#include "actions/Action.hpp"
#include "domain/Position.hpp"

#include <optional>
#include <string>
#include <vector>

enum class TaskType {
    SolveLevel,
    MoveBoxToGoal,
    MoveBoxToCell,
    MoveBoxOneStep,
    MoveAgentTo,
    ClearCell,
    ClearPath,
    PositionAgentForPush,
    PositionAgentForPull,
    Primitive
};

struct Task {
    TaskType type{TaskType::Primitive};

    int agent_id{-1};
    char box{' '};

    Position from{-1, -1};
    Position to{-1, -1};

    std::optional<Action> primitive;

    std::vector<Task> children;

    int priority{0};

    bool is_primitive() const {
        return type == TaskType::Primitive && primitive.has_value();
    }
};

std::string task_type_to_string(TaskType type);
std::string task_to_string(const Task& task);