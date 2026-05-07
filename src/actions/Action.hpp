#pragma once

#include <stdexcept>
#include <string>

// Cardinal directions used by the competition action syntax.
enum class Direction {
    North,
    South,
    East,
    West
};

// Row delta for moving one cell in a direction.
inline int drow(Direction dir) noexcept {
    switch (dir) {
        case Direction::North: return -1;
        case Direction::South: return 1;
        case Direction::East:  return 0;
        case Direction::West:  return 0;
    }
    return 0;
}

// Column delta for moving one cell in a direction.
inline int dcol(Direction dir) noexcept {
    switch (dir) {
        case Direction::North: return 0;
        case Direction::South: return 0;
        case Direction::East:  return 1;
        case Direction::West:  return -1;
    }
    return 0;
}

// Short form expected by the server protocol.
inline char dir_to_char(Direction dir) noexcept {
    switch (dir) {
        case Direction::North: return 'N';
        case Direction::South: return 'S';
        case Direction::East:  return 'E';
        case Direction::West:  return 'W';
    }
    return '?';
}

enum class ActionType {
    Move,
    Push,
    Pull,
    NoOp
};

// One primitive action for a single agent. Push/Pull contain both the agent
// movement direction and the box movement direction because those may differ
// when turning a corner around a box.
struct Action {
    ActionType type{ActionType::NoOp};

    // For Move: move_dir is used.
    // For Push: move_dir = agent direction, box_dir = box direction.
    // For Pull: move_dir = agent direction, box_dir = box movement direction.
    Direction move_dir{Direction::North};
    Direction box_dir{Direction::North};

    static constexpr Action move(Direction dir) noexcept {
        return Action{ActionType::Move, dir, Direction::North};
    }

    static constexpr Action push(Direction agent_dir, Direction box_move_dir) noexcept {
        return Action{ActionType::Push, agent_dir, box_move_dir};
    }

    static constexpr Action pull(Direction agent_dir, Direction box_dir) noexcept {
        return Action{ActionType::Pull, agent_dir, box_dir};
    }

    static constexpr Action noop() noexcept {
        return Action{ActionType::NoOp, Direction::North, Direction::North};
    }

    // Convert to the exact text token consumed by the Java competition server.
    [[nodiscard]] std::string to_string() const {
        switch (type) {
            case ActionType::Move:
                return "Move(" + std::string(1, dir_to_char(move_dir)) + ")";
            case ActionType::Push:
                return "Push(" + std::string(1, dir_to_char(move_dir)) + "," +
                       std::string(1, dir_to_char(box_dir)) + ")";
            case ActionType::Pull:
                return "Pull(" + std::string(1, dir_to_char(move_dir)) + "," +
                       std::string(1, dir_to_char(box_dir)) + ")";
            case ActionType::NoOp:
                return "NoOp";
        }

        throw std::runtime_error("Unknown action type");
    }
};
