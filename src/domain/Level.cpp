#include "domain/Level.hpp"

bool Level::in_bounds(int r, int c) const noexcept {
    return r >= 0 && r < rows && c >= 0 && c < cols;
}

bool Level::is_wall(int r, int c) const noexcept {
    if (!in_bounds(r, c)) {
        return true;
    }
    return grid[r][c] == '+';
}