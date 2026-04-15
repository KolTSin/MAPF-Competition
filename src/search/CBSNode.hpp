#pragma once

#include "plan/AgentPlan.hpp"
#include "plan/Conflicts.hpp"

#include <vector>
#include <optional>

struct CBSNode {
    std::vector<Constraint> constraints;
    std::vector<AgentPlan> plans;
    std::optional<Conflict> first_conflict;
    int makespan = 0;
    int sum_of_costs = 0;
};