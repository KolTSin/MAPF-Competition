#pragma once

#include <cstddef>

struct Position {
    int row{};
    int col{};

    bool operator==(const Position& other) const noexcept {
        return row == other.row && col == other.col;
    }

    bool operator!=(const Position& other) const noexcept {
        return !(*this == other);
    }
};

struct PositionHash {
    std::size_t operator()(const Position& p) const noexcept {
        return (static_cast<std::size_t>(p.row) << 32) ^
               static_cast<std::size_t>(p.col);
    }
};