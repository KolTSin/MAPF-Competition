#pragma once

#include "actions/Action.hpp"
#include "plan/Plan.hpp"

#include <vector>

class PlanMerger {
public:
    static Plan merge_agent_plans(const std::vector<Plan>& agent_plans,
                                  int num_agents);
};