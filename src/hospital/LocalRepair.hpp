#pragma once

#include "domain/Level.hpp"
#include "plan/TaskPlan.hpp"
#include "state/State.hpp"
#include "tasks/Task.hpp"

enum class RepairStageOutcome {
    Delay,
    AlternateAgent,
    AlternateParking,
    NeighborhoodReplan,
    SafePrefixFallback,
    Unresolved
};

struct RepairResult {
    RepairStageOutcome outcome{RepairStageOutcome::Unresolved};
    TaskPlan plan{};
    std::string reason;
};

class LocalRepair {
public:
    [[nodiscard]] RepairResult repair(const Level& level, const State& state, const Task& task, const TaskPlan& failed) const;
    [[nodiscard]] RepairStageOutcome last_outcome() const { return last_outcome_; }
private:
    mutable RepairStageOutcome last_outcome_{RepairStageOutcome::Unresolved};
};
