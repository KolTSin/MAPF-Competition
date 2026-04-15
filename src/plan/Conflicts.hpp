#pragma once

#include "domain/Position.hpp"

#include <vector>
#include <array>
#include <string>
#include <sstream>

// struct ConflictHasher {
//     std::size_t operator()(const Conflict& r) const noexcept;
// };

struct Constraint {
    int agent_id;
    int time;

    Position cell;
    std::array<Position, 2> from{};
    std::array<Position, 2> to{};

    std::string to_string() const {
        std::ostringstream os;
        os << "(a" << agent_id
        << ") @ (" << cell.row
        << "," << cell.col
        << "), t=" << time;
        return os.str();
    }
};

struct Conflict {
    std::vector<int> agents{-1,-1};
    int time;

    Position cell;
    std::array<Position, 2> from{};
    std::array<Position, 2> to{};

    bool operator==(const Conflict& other) const noexcept {
        return cell == other.cell &&
               agents[0] == other.agents[0] &&
               agents[1] == other.agents[1];
    }

    std::string to_string() const {
        std::ostringstream os;
        os << "(a" << agents[0]
        << ", a" << agents[1]
        << ") @ (" << cell.row
        << "," << cell.col
        << "), t=" << time;
        return os.str();
    }
};