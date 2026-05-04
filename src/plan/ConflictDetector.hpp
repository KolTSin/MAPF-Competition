#pragma once

#include "plan/Plan.hpp"
#include "state/State.hpp"

#include <string>

class ConflictDetector {
public:
    [[nodiscard]] static bool has_conflict(const Plan& plan, const State& initial_state, std::string* reason = nullptr);
};
