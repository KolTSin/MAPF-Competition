#pragma once

#include "domain/Position.hpp"

#include <string>

enum class TaskType {
    MoveAgentTo,
    MoveBoxToGoal,
    ClearCell,
    ClearPath,
    PositionAgentForBox,
    PushBox,
    PullBox,
    PrimitiveAction
};

enum class HospitalTaskPhase {
    ClearBlockers = 0,
    SolvePrimaryGoals = 1,
    RestoreRelocatedBoxes = 2
};

struct Task {
    TaskType type{TaskType::MoveAgentTo};
    int agent_id{-1};
    char box_symbol{'\0'};
    Position source{};
    Position destination{};
    int priority{0};
    std::string reason;
    HospitalTaskPhase phase{HospitalTaskPhase::SolvePrimaryGoals};
    Position final_destination{-1, -1};
};
