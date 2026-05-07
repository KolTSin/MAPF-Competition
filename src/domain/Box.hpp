#pragma once

#include "domain/Color.hpp"
#include "domain/Position.hpp"

// A pushable/pullable box. The uppercase id links the box to matching lowercase
// goal cells and to its color entry in Level::box_colors.
struct Box {
    char id{'?'};
    Color color{Color::Unknown};
    Position pos{};
};
