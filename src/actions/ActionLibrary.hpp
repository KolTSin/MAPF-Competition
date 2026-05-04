#pragma once

#include "actions/Action.hpp"

#include <array>

namespace ActionLibrary {

inline constexpr std::array<Action, 29> ALL_ACTIONS = {
    Action::noop(),

    Action::move(Direction::North),
    Action::move(Direction::South),
    Action::move(Direction::East),
    Action::move(Direction::West),

    Action::push(Direction::North, Direction::North),
    Action::push(Direction::North, Direction::East),
    Action::push(Direction::North, Direction::West),

    Action::push(Direction::South, Direction::South),
    Action::push(Direction::South, Direction::East),
    Action::push(Direction::South, Direction::West),

    Action::push(Direction::East, Direction::East),
    Action::push(Direction::East, Direction::North),
    Action::push(Direction::East, Direction::South),

    Action::push(Direction::West, Direction::West),
    Action::push(Direction::West, Direction::North),
    Action::push(Direction::West, Direction::South),

    Action::pull(Direction::North, Direction::East),
    Action::pull(Direction::North, Direction::West),
    Action::pull(Direction::North, Direction::North),

    Action::pull(Direction::South, Direction::East),
    Action::pull(Direction::South, Direction::West),
    Action::pull(Direction::South, Direction::South),

    Action::pull(Direction::East, Direction::North),
    Action::pull(Direction::East, Direction::South),
    Action::pull(Direction::East, Direction::East),

    Action::pull(Direction::West, Direction::North),
    Action::pull(Direction::West, Direction::South),
    Action::pull(Direction::West, Direction::West)
};

} // namespace ActionLibrary