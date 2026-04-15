#pragma once

#include "plan/Conflicts.hpp"
#include "plan/Plan.hpp"
#include "plan/AgentPlan.hpp"
#include "domain/Position.hpp"

#include <optional>
#include <vector>



class ConflictDetector {
public:
    static Conflict findFirstConflict(const std::vector<AgentPlan>& plans);

private:
    static Position getPositionAt(const AgentPlan& plan, int t);
};