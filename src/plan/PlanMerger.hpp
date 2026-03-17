#pragma once

#include "actions/Action.hpp"
#include "plan/Plan.hpp"

#include <vector>

class PlanMerger {
public:
    static Plan merge_agent_plans(const std::vector<std::vector<Action>>& agent_plans,
                                  int num_agents);
};