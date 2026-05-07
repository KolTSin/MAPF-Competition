#pragma once

#include "actions/Action.hpp"
#include "state/State.hpp"

#include <cstddef>

// Search tree node. parent_index points into the owning node vector so the final
// action sequence can be reconstructed without copying whole paths per node.
struct Node {
    State state;
    int time{0}; // Used by space-time searches; plain A* leaves it at zero.

    int g{0};    // Cost from the start node.
    int h{0};    // Heuristic estimate to a goal.

    int parent_index{-1};
    Action action{Action::noop()}; // Action taken from parent to reach this node.

    [[nodiscard]] int f() const noexcept {
        return g + h;
    }
};
