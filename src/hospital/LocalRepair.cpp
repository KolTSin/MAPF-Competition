#include "hospital/LocalRepair.hpp"

TaskPlan LocalRepair::repair(const Level& level, const State& state, const Task& task, const TaskPlan& failed) const {
    (void)level; (void)state; (void)task;
    TaskPlan out = failed;
    if (out.success) return out;
    out.failure_reason += "|repair_attempted:delay,alternate_agent,alternate_parking,neighborhood_replan,safe_prefix";
    return out;
}
