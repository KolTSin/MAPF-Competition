#pragma once

#include "domain/Level.hpp"
#include "plan/Plan.hpp"
#include "state/State.hpp"

// Greedy CBS-style temporal repair for already-planned joint actions.
//
// The high-level solver can first create independent/naive per-agent timelines.
// This repairer then repeatedly finds the earliest agent-agent conflict and
// branches on the two involved agents by inserting a single wait into one
// timeline, keeping the branch that pushes the next conflict furthest into the
// future. It is intentionally bounded and conservative: if repair cannot prove
// that the resulting plan is executable, it returns the original plan so callers
// can fall back to their existing solver strategy.
class PlanConflictRepairer {
public:
    struct Result {
        Plan plan;
        bool changed{false};
        bool conflict_free{false};
        int iterations{0};
    };

    [[nodiscard]] Result repair(const Level& level,
                                const State& initial_state,
                                const Plan& input,
                                int max_iterations = 128,
                                int max_extra_steps = 256) const;
};
