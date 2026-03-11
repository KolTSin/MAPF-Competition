#pragma once

#include "domain/Color.hpp"
#include "domain/Position.hpp"

struct Agent {
    int id{-1};
    Color color{Color::Unknown};
    Position pos{};
};