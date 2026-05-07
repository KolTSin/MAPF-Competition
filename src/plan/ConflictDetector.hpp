#pragma once

#include "plan/Conflicts.hpp"
#include "plan/Plan.hpp"
#include "plan/AgentPlan.hpp"
#include "domain/Position.hpp"
#include "state/State.hpp"

#include <string>
#include <optional>
#include <vector>



class ConflictDetector {
public:
    static Conflict findFirstConflict(const std::vector<AgentPlan>& plans);
    [[nodiscard]] static bool has_conflict(const Plan& plan, const State& initial_state, std::string* reason = nullptr);


private:
    static Position getPositionAt(const AgentPlan& plan, int t);
};


