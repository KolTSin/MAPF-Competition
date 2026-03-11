#pragma once

#include "domain/Position.hpp"
#include <vector>

class State {
public:
    std::vector<Position> agent_positions;
    std::vector<Position> box_positions;
};