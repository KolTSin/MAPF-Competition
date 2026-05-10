#pragma once

#include "domain/Position.hpp"

#include <array>
#include <sstream>
#include <string>
#include <vector>

enum class ConflictType {
    None,

    // Agent-agent conflicts.
    AgentVertex,
    AgentEdgeSwap,
    AgentFollow,

    // Agent-box conflicts.
    AgentIntoBoxStartCell,
    AgentBoxSameDestination,
    BoxIntoAgentStartCell,

    // Box-box / box manipulation conflicts.
    BoxVertex,
    BoxIntoBoxStartCell,
    SameBoxMovedByTwoAgents
};

// struct ConflictHasher {
//     std::size_t operator()(const Conflict& r) const noexcept;
// };

struct Constraint {
    int agent_id{-1};
    int time{-1};
    ConflictType type{ConflictType::None};

    Position cell{-1, -1};
    std::array<Position, 2> from{{Position{-1, -1}, Position{-1, -1}}};
    std::array<Position, 2> to{{Position{-1, -1}, Position{-1, -1}}};

    std::string to_string() const {
        std::ostringstream os;
        os << "(" << static_cast<int>(type) << ", a" << agent_id
           << ") @ (" << cell.row
           << "," << cell.col
           << "), t=" << time;
        return os.str();
    }
};

struct Conflict {
    ConflictType type{ConflictType::None};

    // Agents involved. For box-only conflicts, these are the movers if known.
    std::array<int, 2> agents{{-1, -1}};

    // Time of the conflict.
    // For destination conflicts, this is usually t + 1.
    // For transition conflicts, this is the step t.
    int time{-1};

    // Main conflict cell, when there is one.
    Position cell{-1, -1};

    // Transition info, useful for swaps.
    std::array<Position, 2> from{{Position{-1, -1}, Position{-1, -1}}};
    std::array<Position, 2> to{{Position{-1, -1}, Position{-1, -1}}};

    // Box info. box_ids are synthetic ids assigned by initial scan order.
    std::array<int, 2> boxes{{-1, -1}};
    std::array<char, 2> box_letters{{'\0', '\0'}};

    bool valid() const noexcept {
        return type != ConflictType::None;
    }
};

std::string to_string(ConflictType type);
std::string to_string(const Conflict& c);
