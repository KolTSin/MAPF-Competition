#pragma once

#include "actions/Action.hpp"
#include "domain/Position.hpp"

struct ActionEffect {
    Position agent_from{};
    Position agent_to{};

    bool moves_box{false};
    Position box_from{};
    Position box_to{};
};

class ActionSemantics {
public:
    static ActionEffect compute_effect(const Position& agent_pos, const Action& action);
};