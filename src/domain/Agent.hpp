#pragma once

#include "domain/Color.hpp"
#include "domain/Position.hpp"

// A movable agent from the level file. Agent ids are numeric characters in the
// input and also index the State::agent_positions vector.
struct Agent {
    int id{-1};          // Competition id, usually 0-9.
    Color color{Color::Unknown}; // Determines which boxes this agent may move.
    Position pos{};      // Initial grid position.
};
