#pragma once

#include "state/State.hpp"

#include <string>

// Common interface for admissible or greedy scoring functions used by search.
// Lower values should mean the state is closer to completion.
class IHeuristic {
public:
    virtual ~IHeuristic() = default;
    virtual int evaluate(const State& state) const = 0;
    virtual std::string name() const = 0;
};
