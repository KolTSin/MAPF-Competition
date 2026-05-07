#pragma once

#include "domain/Position.hpp"

// Goal symbol and location parsed from the level goal section. Lowercase
// letters are box goals; digits are agent goals.
struct Goal {
    char symbol{'?'};
    Position pos{};
};
