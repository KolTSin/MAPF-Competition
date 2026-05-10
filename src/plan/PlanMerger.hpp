#pragma once

#include "actions/Action.hpp"
#include "plan/Plan.hpp"
#include "plan/AgentPlan.hpp"
#include "domain/Level.hpp"
#include "state/State.hpp"

#include <vector>

// Pads individual agent plans with NoOps and transposes them into timesteps.
class PlanMerger {
public:
    static Plan merge_agent_plans(const std::vector<AgentPlan>& agent_plans,
                                  int num_agents);

    // Remove global idle timesteps after plans have been transposed.  This is
    // safe because a row where every agent NoOps leaves the world unchanged, so
    // later actions can begin one tick earlier without changing relative agent
    // or box motion.
    static void compress_idle_steps(Plan& plan);

    // Greedily left-shift individual agent actions across their own NoOps when
    // the resulting full joint plan remains executable and conflict-free from
    // the provided initial state. This preserves each agent's action order while
    // allowing independent tasks to overlap in time.
    static void compact_independent_actions(const Level& level, const State& initial_state, Plan& plan);
};
