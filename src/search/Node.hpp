#pragma once

#include "actions/Action.hpp"
#include "state/State.hpp"

#include <cstddef>

struct Node {
    State state;

    int g{0};
    int h{0};

    int parent_index{-1};
    Action action{Action::noop()};

    [[nodiscard]] int f() const noexcept {
        return g + h;
    }
};