#pragma once

#include "state/State.hpp"
#include <string>

class IHeuristic {
public:
    virtual ~IHeuristic() = default;
    virtual int evaluate(const State& state) const = 0;
    virtual std::string name() const = 0;
};