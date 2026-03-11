#pragma once

#include "domain/Agent.hpp"
#include "domain/Box.hpp"
#include "domain/Goal.hpp"
#include <string>
#include <vector>

class Level {
public:
    int rows{0};
    int cols{0};
    std::vector<std::string> grid;
    std::vector<Agent> agents;
    std::vector<Box> boxes;
    std::vector<Goal> goals;

    [[nodiscard]] bool in_bounds(int r, int c) const noexcept;
    [[nodiscard]] bool is_wall(int r, int c) const noexcept;
};