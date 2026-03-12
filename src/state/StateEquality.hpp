#pragma once

#include "state/State.hpp"

struct StateEqual {
    bool operator()(const State& a, const State& b) const noexcept {
        return a.agent_positions == b.agent_positions &&
               a.box_pos == b.box_pos;
    }
};