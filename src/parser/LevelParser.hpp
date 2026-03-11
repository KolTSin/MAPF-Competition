#pragma once

#include "domain/Level.hpp"
#include "state/State.hpp"
#include <istream>

struct ParsedLevel {
    Level level;
    State initial_state;
};

class LevelParser {
public:
    static ParsedLevel parse(std::istream& in);
};