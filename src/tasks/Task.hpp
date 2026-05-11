#pragma once

#include "domain/Position.hpp"

#include <string>
#include <vector>

// High-level work item types used by the HTN-style competitive solver.
enum class TaskType {
    DeliverBoxToGoal,
    MoveBlockingBoxToParking,
    MoveAgentToGoal,
    ParkAgentSafely,
};

// Declarative task description. Planners translate this into primitive actions,
// while the scheduler uses dependencies and priority to decide ordering.
struct Task {
    TaskType type{TaskType::DeliverBoxToGoal};
    int task_id{-1};
    int agent_id{-1};

    char box_id{'\0'};
    Position box_pos{};

    char goal_symbol{'\0'};
    Position goal_pos{};

    Position parking_pos{};

    // Ranked parking alternatives for blocker relocation. parking_pos remains
    // the preferred/default target for backwards compatibility, while the
    // scheduler may retry later entries when the first target is not plannable
    // under current reservations or push-side constraints.
    std::vector<Position> parking_candidates{};

    int priority{0};
    std::vector<int> dependencies{};

    // Structured blocker relationship metadata. For relocation tasks,
    // unblocks_box_id identifies the delivery box that cannot be planned until
    // this blocker has been moved out of the way. Keep debug_label diagnostic-only.
    char unblocks_box_id{'\0'};
    std::string debug_label{};

    [[nodiscard]] std::string describe() const;
};
