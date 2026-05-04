#pragma once

#include "domain/Position.hpp"

#include <string>

enum class TaskType {
    DeliverBoxToGoal,
    MoveBlockingBoxToParking,
    MoveAgentToGoal,
};

struct Task {
    TaskType type{TaskType::DeliverBoxToGoal};
    int task_id{-1};
    int agent_id{-1};

    char box_id{'\0'};
    Position box_pos{};

    char goal_symbol{'\0'};
    Position goal_pos{};

    Position parking_pos{};
    int priority{0};

    [[nodiscard]] std::string describe() const;
};
