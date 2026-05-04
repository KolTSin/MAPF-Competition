#include "state/State.hpp"
#include <assert.h>

int State::index(int r, int c) const noexcept {
    return r * cols + c;
}

bool State::in_bounds(int r, int c) const noexcept {
    return r >= 0 && r < rows && c >= 0 && c < cols;
}

int State::num_agents() const noexcept {
    return static_cast<int>(agent_positions.size());
}

char State::box_at(int r, int c) const noexcept {
    if (!in_bounds(r, c)) {
        return '\0';
    }
    return box_pos[index(r, c)];
}

bool State::has_box(int r, int c) const noexcept {
    return box_at(r, c) != '\0';
}

void State::set_box(int r, int c, char value) {
    assert(in_bounds(r, c));
    box_pos[index(r, c)] = value;
}