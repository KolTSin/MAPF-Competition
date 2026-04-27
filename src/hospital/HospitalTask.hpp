#pragma once

#include "domain/Position.hpp"

#include <string>

enum class HospitalTaskType {
    AssignBox,
    RelocateBox,
    TransportBox
};

enum class HospitalTaskPhase {
    ClearBlockers = 0,
    SolvePrimaryGoals = 1,
    RestoreRelocatedBoxes = 2
};

struct HospitalTask {
    HospitalTaskType type{HospitalTaskType::TransportBox};
    int agent_id{-1};
    char box_symbol{'\0'};
    Position source{};
    Position destination{};
    int priority{0};
    std::string reason;
    HospitalTaskPhase phase{HospitalTaskPhase::SolvePrimaryGoals};
    Position final_destination{-1, -1};
};
