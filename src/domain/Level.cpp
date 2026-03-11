#include "domain/Level.hpp"

Level::Level() {
    agent_colors.fill(Color::Unknown);
    box_colors.fill(Color::Unknown);
}

int Level::index(int r, int c) const noexcept {
    return r * cols + c;
}

bool Level::in_bounds(int r, int c) const noexcept {
    return r >= 0 && r < rows && c >= 0 && c < cols;
}

bool Level::is_wall(int r, int c) const noexcept {
    if (!in_bounds(r, c)) {
        return true;
    }
    return walls[index(r, c)];
}

char Level::goal_at(int r, int c) const noexcept {
    if (!in_bounds(r, c)) {
        return '\0';
    }
    return goals[index(r, c)];
}