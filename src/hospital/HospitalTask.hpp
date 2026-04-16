#pragma once

#include "domain/Position.hpp"

#include <string>

enum class HospitalTaskType {
    AssignBox,
    RelocateBox,
    TransportBox
};

struct HospitalTask {
    HospitalTaskType type{HospitalTaskType::TransportBox};
    int agent_id{-1};
    char box_symbol{'\0'};
    Position source{};
    Position destination{};
    int priority{0};
    std::string reason;
};
