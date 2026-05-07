#pragma once

#include "plan/Plan.hpp"
#include "state/State.hpp"

#include <string>

// Simulates a joint plan and reports vertex/edge/resource conflicts before the
// client sends the plan to the server.
class ConflictDetector {
public:
    [[nodiscard]] static bool has_conflict(const Plan& plan, const State& initial_state, std::string* reason = nullptr);
};
