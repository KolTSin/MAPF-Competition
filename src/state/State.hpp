#pragma once

#include "domain/Position.hpp"

#include <vector>

// Dynamic part of a search node. Static data such as walls, goals, and colors
// lives in Level; State only stores where agents and boxes currently are.
class State {
public:
    int rows{0};
    int cols{0};

    // agent_positions[id] gives the current cell of that agent.
    std::vector<Position> agent_positions;

    // Flattened rows*cols board. '\0' means no box; otherwise stores 'A'..'Z'.
    std::vector<char> box_pos;

    [[nodiscard]] int index(int r, int c) const noexcept;
    [[nodiscard]] bool in_bounds(int r, int c) const noexcept;
    [[nodiscard]] int num_agents() const noexcept;

    [[nodiscard]] char box_at(int r, int c) const noexcept;
    [[nodiscard]] bool has_box(int r, int c) const noexcept;

    void set_box(int r, int c, char value);
};
