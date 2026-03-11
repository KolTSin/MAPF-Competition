#pragma once

#include <string>

enum class ActionType {
    Move,
    Push,
    Pull,
    NoOp
};

struct Action {
    ActionType type{ActionType::NoOp};
    std::string command{"NoOp"};
};