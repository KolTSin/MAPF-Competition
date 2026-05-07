#pragma once

#include <cstddef>
#include <sstream>

// Immutable-style row/column coordinate used throughout the grid code. The
// project consistently treats row as y and column as x.
struct Position {
    int row{-1};
    int col{-1};

    bool operator==(const Position& other) const noexcept {
        return row == other.row && col == other.col;
    }

    bool operator!=(const Position& other) const noexcept {
        return !(*this == other);
    }

    std::string to_string() const {
        std::ostringstream os;
        os << "(" << row << ", " << col << ")";
        return os.str();
    }
};

// Hash helper so Position can be used as a key in unordered containers.
struct PositionHash {
    std::size_t operator()(const Position& p) const noexcept {
        return (static_cast<std::size_t>(p.row) << 32) ^
               static_cast<std::size_t>(p.col);
    }
};
