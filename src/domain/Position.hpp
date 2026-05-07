#pragma once

#include <cstddef>
#include <sstream>

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

struct PositionHash {
    std::size_t operator()(const Position& p) const noexcept {
        return (static_cast<std::size_t>(p.row) << 32) ^
               static_cast<std::size_t>(p.col);
    }
};