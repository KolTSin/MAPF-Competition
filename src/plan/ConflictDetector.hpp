#pragma once

#include "plan/AgentPlan.hpp"
#include "plan/Conflicts.hpp"
#include "plan/Plan.hpp"
#include "state/State.hpp"

#include <string>
#include <vector>

class ConflictDetector {
public:
    static Position getPositionAt(const AgentPlan& plan, int t);

    // Agent-only version, useful for CBS over AgentPlan trajectories.
    static std::vector<Conflict> findAllAgentConflicts(const std::vector<AgentPlan>& plans);
    static Conflict findFirstConflict(const std::vector<AgentPlan>& plans);

    // Full hospital-domain version: agents + boxes.
    static std::vector<Conflict> findAllConflicts(
        const Plan& plan,
        const State& initial_state,
        bool stop_at_first_conflicting_timestep = true
    );
    static std::vector<Conflict> findAllConflicts(
        const std::vector<AgentPlan>& plans,
        const State& initial_state,
        bool stop_at_first_conflicting_timestep = true
    );

    static Conflict findFirstConflict(
        const Plan& plan,
        const State& initial_state
    );
    static Conflict findFirstConflict(
        const std::vector<AgentPlan>& plans,
        const State& initial_state
    );

    static bool has_conflict(
        const Plan& plan,
        const State& initial_state,
        std::string* reason = nullptr
    );
};
