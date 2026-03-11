#pragma once

#include "domain/Agent.hpp"
#include "domain/Box.hpp"
#include "domain/Goal.hpp"
#include <string>
#include <vector>
#include <array>

class Level {
public:
    // level name
    std::string name;

    // number of rows and columns in level
    int rows{0};
    int cols{0};

    // walls and goals
    std::vector<bool> walls;
    std::vector<char> goals;

    // colors
    std::array<Color, 10> agent_colors;
    std::array<Color, 26> box_colors;

    // construct level
    Level();

    // define member functions
    [[nodiscard]] int index(int r, int c) const noexcept;
    [[nodiscard]] bool in_bounds(int r, int c) const noexcept;
    [[nodiscard]] bool is_wall(int r, int c) const noexcept;
    [[nodiscard]] char goal_at(int r, int c) const noexcept;
};