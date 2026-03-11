#pragma once

#include "domain/Position.hpp"
#include <vector>

class State {
public:
    int rows{0};
    int cols{0};

    std::vector<Position> agent_positions;
    std::vector<char> box_pos;

    [[nodiscard]] int index(int r, int c) const noexcept;
    [[nodiscard]] bool in_bounds(int r, int c) const noexcept;
    [[nodiscard]] int num_agents() const noexcept;

    [[nodiscard]] char box_at(int r, int c) const noexcept;
    [[nodiscard]] bool has_box(int r, int c) const noexcept;

    void set_box(int r, int c, char value);
};