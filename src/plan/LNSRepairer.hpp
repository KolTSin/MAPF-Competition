#pragma once

#include "domain/Level.hpp"
#include "plan/AgentPlan.hpp"
#include "plan/Plan.hpp"
#include "state/State.hpp"
#include "tasks/Task.hpp"
#include "utils/PlanningDeadline.hpp"

#include <vector>

// Large-neighborhood repair for one HTN scheduling wave.
//
// The repairer treats a conflicted wave as an LNS incumbent: it detects the
// earliest conflict cluster, destroys only the involved agents/boxes, freezes
// every other timeline as reservations, replans the selected task neighborhood,
// and accepts the candidate when it decreases conflicts or preserves the solved
// goal count while removing the first conflict.
class LNSRepairer {
public:
    struct Result {
        Plan plan;
        std::vector<AgentPlan> agent_plans;
        bool changed{false};
        bool conflict_free{false};
        int conflicts_before{0};
        int conflicts_after{0};
        int iterations{0};
    };

    [[nodiscard]] Result repair_wave(const Level& level,
                                     const State& initial_state,
                                     const std::vector<Task>& tasks,
                                     const std::vector<AgentPlan>& incumbent,
                                     const PlanningDeadline& deadline,
                                     int max_iterations = 12) const;
};
