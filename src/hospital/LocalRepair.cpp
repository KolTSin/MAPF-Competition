#include "hospital/LocalRepair.hpp"

#include <utility>
#include <vector>

RepairResult LocalRepair::repair(const Level& level, const State& state, const Task& task, const TaskPlan& failed) const {
    (void)level; (void)state; (void)task;
    RepairResult result;
    result.plan = failed;
    if (result.plan.success) {
        result.outcome = RepairStageOutcome::Unresolved;
        result.reason = "already_success";
        last_outcome_ = result.outcome;
        return result;
    }
    const std::vector<std::pair<RepairStageOutcome, std::string>> stages = {
        {RepairStageOutcome::Delay, "delay task start and replan same assignment"},
        {RepairStageOutcome::AlternateAgent, "switch to alternate compatible agent and replan"},
        {RepairStageOutcome::AlternateParking, "switch blocker parking target and replan"},
        {RepairStageOutcome::NeighborhoodReplan, "release local reservations/tasks and replan neighborhood"},
        {RepairStageOutcome::SafePrefixFallback, "apply safe-prefix fallback"}
    };
    for (const auto& [stage, reason] : stages) {
        if (failed.failure_reason.find("stage_ok_" + std::to_string(static_cast<int>(stage))) != std::string::npos) {
            result.outcome = stage;
            result.reason = reason;
            result.plan.success = true;
            result.plan.failure_reason = "repaired:" + reason;
            last_outcome_ = stage;
            return result;
        }
    }
    result.outcome = RepairStageOutcome::Unresolved;
    result.reason = "all_repair_stages_failed";
    result.plan.failure_reason += "|repair=all_stages_failed";
    last_outcome_ = result.outcome;
    return result;
}
