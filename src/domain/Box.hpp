#pragma once

#include "domain/Color.hpp"
#include "domain/Position.hpp"

struct Box {
    char id{'?'};
    Color color{Color::Unknown};
    Position pos{};
};