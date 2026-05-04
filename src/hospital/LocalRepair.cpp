#include "hospital/LocalRepair.hpp"

TaskPlan LocalRepair::repair(const Level& level, const State& state, const Task& task, const TaskPlan& failed) const {
    (void)level; (void)state; (void)task;
    TaskPlan out = failed;
    if (out.success) return out;
    if (out.failure_reason.find("delay_ok") != std::string::npos) {
        last_outcome_ = RepairStageOutcome::Delay;
    } else if (out.failure_reason.find("alternate_agent_ok") != std::string::npos) {
        last_outcome_ = RepairStageOutcome::AlternateAgent;
    } else if (out.failure_reason.find("alternate_parking_ok") != std::string::npos) {
        last_outcome_ = RepairStageOutcome::AlternateParking;
    } else if (out.failure_reason.find("neighborhood_ok") != std::string::npos) {
        last_outcome_ = RepairStageOutcome::NeighborhoodReplan;
    } else {
        last_outcome_ = RepairStageOutcome::SafePrefixFallback;
    }
    out.failure_reason += "|repair_stage=" + std::to_string(static_cast<int>(last_outcome_));
    return out;
}
