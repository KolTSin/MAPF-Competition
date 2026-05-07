#pragma once

#include "actions/Action.hpp"
#include "plan/Plan.hpp"
#include "plan/AgentPlan.hpp"

#include <vector>

// Pads individual agent plans with NoOps and transposes them into timesteps.
class PlanMerger {
public:
    static Plan merge_agent_plans(const std::vector<AgentPlan>& agent_plans,
                                  int num_agents);
};
