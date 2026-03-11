#pragma once

#include "actions/JointAction.hpp"
#include <vector>

class Plan {
public:
    std::vector<JointAction> steps;

    [[nodiscard]] bool empty() const noexcept {
        return steps.empty();
    }
};