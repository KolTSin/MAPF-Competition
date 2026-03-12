#pragma once

#include <stdexcept>
#include <string>

enum class Direction {
    North,
    South,
    East,
    West
};

inline int drow(Direction dir) noexcept {
    switch (dir) {
        case Direction::North: return -1;
        case Direction::South: return 1;
        case Direction::East:  return 0;
        case Direction::West:  return 0;
    }
    return 0;
}

inline int dcol(Direction dir) noexcept {
    switch (dir) {
        case Direction::North: return 0;
        case Direction::South: return 0;
        case Direction::East:  return 1;
        case Direction::West:  return -1;
    }
    return 0;
}

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

struct Action {
    ActionType type{ActionType::NoOp};

    // For Move: move_dir is used
    // For Push: move_dir = agent direction, box_dir = box direction
    // For Pull: move_dir = agent direction, box_dir = direction from agent to box before move
    Direction move_dir{Direction::North};
    Direction box_dir{Direction::North};

    static Action move(Direction dir) noexcept {
        return Action{ActionType::Move, dir, Direction::North};
    }

    static Action push(Direction agent_dir, Direction box_move_dir) noexcept {
        return Action{ActionType::Push, agent_dir, box_move_dir};
    }

    static Action pull(Direction agent_dir, Direction box_dir) noexcept {
        return Action{ActionType::Pull, agent_dir, box_dir};
    }

    static Action noop() noexcept {
        return Action{ActionType::NoOp, Direction::North, Direction::North};
    }

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