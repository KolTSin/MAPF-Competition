#pragma once

#include "domain/Position.hpp"

struct Task {
    int agent_id{-1};
    char goal_symbol{'\0'};
    Position goal_pos{};
};