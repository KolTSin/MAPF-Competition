#pragma once

#include "domain/Level.hpp"
#include "plan/AgentPlan.hpp"
#include "plan/Plan.hpp"
#include "state/State.hpp"

// Greedy CBS-style temporal repair for already-planned joint actions.
//
// The high-level solver can first create independent/naive per-agent timelines.
// This repairer then repeatedly finds the earliest conflict and
// branches on the involved agents by inserting waits or bounded rejoining
// detours made from any applicable primitive actions, keeping the branch that
// pushes the next conflict furthest into the future. It is intentionally bounded
// and conservative: if repair cannot prove
// that the resulting plan is executable, it returns the original plan so callers
// can fall back to their existing solver strategy.
class PlanConflictRepairer {
public:
    struct Result {
        Plan plan;
        std::vector<AgentPlan> agent_plans;
        bool changed{false};
        bool conflict_free{false};
        int iterations{0};
    };

    [[nodiscard]] Result repair(const Level& level,
                                const State& initial_state,
                                const Plan& input,
                                int max_iterations = 128,
                                int max_extra_steps = 256) const;
    [[nodiscard]] Result repair(const Level& level,
                                const State& initial_state,
                                const std::vector<AgentPlan>& input,
                                int max_iterations = 128,
                                int max_extra_steps = 256) const;
};
