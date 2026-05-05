#pragma once

#include "domain/Position.hpp"

#include <string>
#include <vector>

enum class TaskType {
    DeliverBoxToGoal,
    MoveBlockingBoxToParking,
    MoveAgentToGoal,
    ParkAgentSafely,
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
    std::vector<int> dependencies{};
    std::string debug_label{};

    [[nodiscard]] std::string describe() const;
};
