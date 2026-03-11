#pragma once

#include "domain/Level.hpp"
#include <istream>

class LevelParser {
public:
    static Level parse(std::istream& in);
};