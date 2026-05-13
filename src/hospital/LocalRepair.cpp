#include "hospital/LocalRepair.hpp"

#include "hospital/AgentPathPlanner.hpp"
#include "hospital/BoxTransportPlanner.hpp"

#include <algorithm>
#include <cstdlib>
#include <sstream>
#include <utility>
#include <vector>

namespace {

bool is_box_task(TaskType type) {
    return type == TaskType::DeliverBoxToGoal || type == TaskType::MoveBlockingBoxToParking;
}

bool is_agent_task(TaskType type) {
    return type == TaskType::MoveAgentToGoal || type == TaskType::ParkAgentSafely;
}

int manhattan(Position a, Position b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

bool empty_plan_is_valid(const Task& task, const State& state) {
    if (task.type == TaskType::DeliverBoxToGoal || task.type == TaskType::MoveBlockingBoxToParking) {
        return task.box_pos == task.goal_pos;
    }
    if (is_agent_task(task.type)) {
        return task.agent_id >= 0 && task.agent_id < state.num_agents() && state.agent_positions[task.agent_id] == task.goal_pos;
    }
    return false;
}

TaskPlan plan_task(const Level& level,
                   const State& state,
                   const Task& task,
                   const ReservationTable& reservations,
                   int start_time,
                   const PlanningDeadline& deadline,
                   const SolverConfig& config) {
    if (is_agent_task(task.type)) {
        AgentPathPlanner planner;
        return planner.plan(level, state, task, reservations, start_time, deadline);
    }
    BoxTransportPlanner planner(config.max_box_planner_expansions);
    return planner.plan(level, state, task, reservations, start_time, deadline);
}

bool acceptable_repair_plan(const Task& task, const State& state, const TaskPlan& plan) {
    return plan.success && (!plan.agent_plan.actions.empty() || empty_plan_is_valid(task, state));
}

std::vector<int> alternate_agents(const Level& level, const State& state, const Task& task) {
    std::vector<std::pair<int, int>> scored;
    if (!is_box_task(task.type) || task.box_id < 'A' || task.box_id > 'Z') return {};

    const Color box_color = level.box_colors[static_cast<std::size_t>(task.box_id - 'A')];
    for (int agent = 0; agent < state.num_agents(); ++agent) {
        if (agent == task.agent_id) continue;
        if (level.agent_colors[static_cast<std::size_t>(agent)] != box_color) continue;
        scored.emplace_back(manhattan(state.agent_positions[static_cast<std::size_t>(agent)], task.box_pos), agent);
    }
    std::sort(scored.begin(), scored.end());

    std::vector<int> out;
    out.reserve(scored.size());
    for (const auto& [_, agent] : scored) out.push_back(agent);
    return out;
}

bool cell_has_agent(const State& state, Position pos) {
    for (const Position agent : state.agent_positions) {
        if (agent == pos) return true;
    }
    return false;
}

std::vector<Position> alternate_parking_cells(const Level& level, const State& state, const Task& task) {
    std::vector<Position> candidates;
    if (task.type != TaskType::MoveBlockingBoxToParking && task.type != TaskType::ParkAgentSafely) return candidates;

    const Position original = task.parking_pos.row >= 0 ? task.parking_pos : task.goal_pos;
    const Position origin = is_box_task(task.type) ? task.box_pos : state.agent_positions[static_cast<std::size_t>(task.agent_id)];

    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const Position pos{r, c};
            if (pos == original) continue;
            if (!level.in_bounds(r, c) || level.is_wall(r, c)) continue;
            if (level.goal_at(r, c) != '\0') continue;
            if (state.has_box(r, c) && pos != task.box_pos) continue;
            if (cell_has_agent(state, pos) && !(is_agent_task(task.type) && pos == state.agent_positions[task.agent_id])) continue;
            candidates.push_back(pos);
        }
    }

    std::stable_sort(candidates.begin(), candidates.end(), [&](Position a, Position b) {
        const int da = manhattan(origin, a);
        const int db = manhattan(origin, b);
        if (da != db) return da < db;
        if (a.row != b.row) return a.row < b.row;
        return a.col < b.col;
    });
    if (candidates.size() > 16) candidates.resize(16);
    return candidates;
}

void append_attempt(std::ostringstream& attempts, const char* stage, const TaskPlan& plan) {
    attempts << stage << ':' << (plan.success ? "success" : (plan.failure_reason.empty() ? "failed" : plan.failure_reason)) << ';';
}

RepairResult make_success(RepairStageOutcome outcome, TaskPlan plan, std::string reason) {
    RepairResult result;
    result.outcome = outcome;
    result.reason = std::move(reason);
    result.plan = std::move(plan);
    result.plan.failure_reason = "repaired:" + result.reason;
    return result;
}

} // namespace

RepairResult LocalRepair::repair(const Level& level, const State& state, const Task& task, const TaskPlan& failed) const {
    ReservationTable empty_reservations;
    return repair(level, state, task, failed, empty_reservations, 0);
}

RepairResult LocalRepair::repair(const Level& level,
                                 const State& state,
                                 const Task& task,
                                 const TaskPlan& failed,
                                 const ReservationTable& reservations,
                                 int start_time) const {
    PlanningDeadline no_deadline;
    return repair(level, state, task, failed, reservations, start_time, no_deadline);
}

RepairResult LocalRepair::repair(const Level& level,
                                 const State& state,
                                 const Task& task,
                                 const TaskPlan& failed,
                                 const ReservationTable& reservations,
                                 int start_time,
                                 const PlanningDeadline& deadline) const {
    RepairResult result;
    result.plan = failed;
    if (deadline.expired()) {
        result.outcome = RepairStageOutcome::Unresolved;
        result.reason = "planning_deadline_expired";
        result.plan.failure_reason = "planning_deadline_expired";
        last_outcome_ = result.outcome;
        return result;
    }
    if (result.plan.success) {
        result.outcome = RepairStageOutcome::Unresolved;
        result.reason = "already_success";
        last_outcome_ = result.outcome;
        return result;
    }

    std::ostringstream attempts;

    for (int delay = 1; delay <= 5; ++delay) {
        if (deadline.expired()) break;
        TaskPlan delayed = plan_task(level, state, task, reservations, start_time + delay, deadline, config_);
        append_attempt(attempts, "delay", delayed);
        if (acceptable_repair_plan(task, state, delayed)) {
            result = make_success(RepairStageOutcome::Delay,
                                  std::move(delayed),
                                  "delay_start_to_t" + std::to_string(start_time + delay));
            last_outcome_ = result.outcome;
            return result;
        }
    }

    for (const int agent : alternate_agents(level, state, task)) {
        if (deadline.expired()) break;
        Task alternate = task;
        alternate.agent_id = agent;
        TaskPlan plan = plan_task(level, state, alternate, reservations, start_time, deadline, config_);
        append_attempt(attempts, "alternate_agent", plan);
        if (acceptable_repair_plan(alternate, state, plan)) {
            result = make_success(RepairStageOutcome::AlternateAgent,
                                  std::move(plan),
                                  "alternate_agent=" + std::to_string(agent));
            last_outcome_ = result.outcome;
            return result;
        }
    }

    for (const Position parking : alternate_parking_cells(level, state, task)) {
        if (deadline.expired()) break;
        Task alternate = task;
        alternate.parking_pos = parking;
        alternate.goal_pos = parking;
        TaskPlan plan = plan_task(level, state, alternate, reservations, start_time, deadline, config_);
        append_attempt(attempts, "alternate_parking", plan);
        if (acceptable_repair_plan(alternate, state, plan)) {
            result = make_success(RepairStageOutcome::AlternateParking,
                                  std::move(plan),
                                  "alternate_parking=(" + std::to_string(parking.row) + "," + std::to_string(parking.col) + ")");
            last_outcome_ = result.outcome;
            return result;
        }
    }

    if (!reservations.empty() || start_time != 0) {
        ReservationTable relaxed;
        TaskPlan replanned = plan_task(level, state, task, relaxed, 0, deadline, config_);
        append_attempt(attempts, "neighborhood_replan", replanned);
        if (acceptable_repair_plan(task, state, replanned)) {
            result = make_success(RepairStageOutcome::NeighborhoodReplan,
                                  std::move(replanned),
                                  "relaxed_local_reservations");
            last_outcome_ = result.outcome;
            return result;
        }
    }

    if (!failed.agent_plan.actions.empty() && failed.agent_plan.positions.size() >= 2) {
        TaskPlan prefix = failed;
        prefix.success = true;
        prefix.agent_plan.actions.resize(1);
        prefix.agent_plan.positions.resize(2);
        if (prefix.box_trajectory.size() > 2) prefix.box_trajectory.resize(2);
        result = make_success(RepairStageOutcome::SafePrefixFallback,
                              std::move(prefix),
                              "safe_prefix_length=1");
        last_outcome_ = result.outcome;
        return result;
    }

    result.outcome = RepairStageOutcome::Unresolved;
    result.reason = "all_repair_stages_failed";
    result.plan.failure_reason += "|repair_attempts=" + attempts.str() + "repair=all_stages_failed";
    last_outcome_ = result.outcome;
    return result;
}
