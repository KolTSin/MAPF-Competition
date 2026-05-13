#include "plan/LNSRepairer.hpp"

#include "actions/ActionSemantics.hpp"
#include "hospital/AgentPathPlanner.hpp"
#include "hospital/BoxTransportPlanner.hpp"
#include "plan/ConflictDetector.hpp"
#include "plan/PlanConflictRepairer.hpp"
#include "plan/PlanMerger.hpp"
#include "plan/ReservationTable.hpp"
#include "plan/WaveProgressGuard.hpp"

#include <algorithm>
#include <set>

namespace {

Plan uncompressed_plan_from_agent_plans(const std::vector<AgentPlan>& plans, int num_agents) {
    std::size_t horizon = 0;
    for (const AgentPlan& plan : plans) horizon = std::max(horizon, plan.actions.size());

    Plan out;
    out.steps.resize(horizon);
    for (std::size_t t = 0; t < horizon; ++t) {
        out.steps[t].actions.resize(static_cast<std::size_t>(num_agents), Action::noop());
        for (int agent = 0; agent < num_agents && agent < static_cast<int>(plans.size()); ++agent) {
            const AgentPlan& plan = plans[static_cast<std::size_t>(agent)];
            if (t < plan.actions.size()) out.steps[t].actions[static_cast<std::size_t>(agent)] = plan.actions[t];
        }
    }
    return out;
}

int count_conflicts(const std::vector<AgentPlan>& plans, const State& state) {
    ConflictDetector detector;
    return static_cast<int>(detector.findAllConflicts(plans, state, false).size());
}

bool contains_int(const std::vector<int>& values, int needle) {
    return std::find(values.begin(), values.end(), needle) != values.end();
}

bool conflict_mentions_task(const Conflict& conflict, const Task& task) {
    if (task.agent_id >= 0 && (conflict.agents[0] == task.agent_id || conflict.agents[1] == task.agent_id)) return true;
    if (task.box_id != '\0' && (conflict.box_letters[0] == task.box_id || conflict.box_letters[1] == task.box_id)) return true;
    return false;
}

std::vector<int> select_neighborhood(const std::vector<Task>& tasks, const Conflict& conflict, int num_agents) {
    std::vector<int> selected;
    auto add = [&](int agent) {
        if (agent >= 0 && agent < num_agents && !contains_int(selected, agent)) selected.push_back(agent);
    };

    add(conflict.agents[0]);
    add(conflict.agents[1]);
    for (const Task& task : tasks) {
        if (conflict_mentions_task(conflict, task)) add(task.agent_id);
    }
    return selected;
}

ReservationTable freeze_unselected(const std::vector<AgentPlan>& plans, const std::vector<int>& selected) {
    ReservationTable reservations;
    for (int agent = 0; agent < static_cast<int>(plans.size()); ++agent) {
        if (contains_int(selected, agent)) continue;
        if (plans[static_cast<std::size_t>(agent)].valid()) reservations.reserve_path(plans[static_cast<std::size_t>(agent)]);
    }
    return reservations;
}

void replay_plan_into_state(State& state, int agent, const AgentPlan& plan) {
    Position cur = state.agent_positions[static_cast<std::size_t>(agent)];
    for (const Action& action : plan.actions) {
        const ActionEffect eff = ActionSemantics::compute_effect(cur, action);
        cur = eff.agent_to;
        if (eff.moves_box && state.in_bounds(eff.box_from.row, eff.box_from.col) && state.in_bounds(eff.box_to.row, eff.box_to.col)) {
            const char moved = state.box_at(eff.box_from.row, eff.box_from.col);
            if (moved != '\0') {
                state.set_box(eff.box_from.row, eff.box_from.col, '\0');
                state.set_box(eff.box_to.row, eff.box_to.col, moved);
            }
        }
    }
    state.agent_positions[static_cast<std::size_t>(agent)] = cur;
}

TaskPlan plan_task(const Level& level,
                   const State& state,
                   const Task& task,
                   const ReservationTable& reservations,
                   const PlanningDeadline& deadline) {
    if (task.type == TaskType::MoveAgentToGoal || task.type == TaskType::ParkAgentSafely) {
        AgentPathPlanner planner;
        return planner.plan(level, state, task, reservations, 0, deadline);
    }
    BoxTransportPlanner planner;
    return planner.plan(level, state, task, reservations, 0, deadline);
}

std::vector<Task> tasks_for_neighborhood(const std::vector<Task>& tasks, const Conflict& conflict, const std::vector<int>& selected) {
    std::vector<Task> out;
    for (const Task& task : tasks) {
        if (contains_int(selected, task.agent_id) || conflict_mentions_task(conflict, task)) out.push_back(task);
    }
    std::stable_sort(out.begin(), out.end(), [](const Task& a, const Task& b) {
        if (a.priority != b.priority) return a.priority > b.priority;
        return a.task_id < b.task_id;
    });
    return out;
}

std::vector<AgentPlan> replan_neighborhood(const Level& level,
                                           const State& initial_state,
                                           const std::vector<Task>& tasks,
                                           const std::vector<AgentPlan>& incumbent,
                                           const Conflict& conflict,
                                           const std::vector<int>& selected,
                                           const PlanningDeadline& deadline) {
    std::vector<AgentPlan> candidate = incumbent;
    candidate.resize(static_cast<std::size_t>(initial_state.num_agents()));
    for (int agent = 0; agent < initial_state.num_agents(); ++agent) {
        AgentPlan& plan = candidate[static_cast<std::size_t>(agent)];
        plan.agent = agent;
        if (contains_int(selected, agent)) {
            plan.actions.clear();
            plan.positions = {initial_state.agent_positions[static_cast<std::size_t>(agent)]};
        } else if (plan.positions.empty()) {
            plan.positions = {initial_state.agent_positions[static_cast<std::size_t>(agent)]};
        }
    }

    ReservationTable reservations = freeze_unselected(incumbent, selected);
    State simulated = initial_state;
    for (Task task : tasks_for_neighborhood(tasks, conflict, selected)) {
        if (deadline.expired()) break;
        if (task.agent_id < 0 || task.agent_id >= initial_state.num_agents()) continue;
        if (!contains_int(selected, task.agent_id)) continue;

        TaskPlan plan = plan_task(level, simulated, task, reservations, deadline);
        if (!plan.success) continue;

        AgentPlan& target = candidate[static_cast<std::size_t>(task.agent_id)];
        if (!target.actions.empty()) target.actions.push_back(Action::noop());
        target.actions.insert(target.actions.end(), plan.agent_plan.actions.begin(), plan.agent_plan.actions.end());

        target.positions.clear();
        Position cur = initial_state.agent_positions[static_cast<std::size_t>(task.agent_id)];
        target.positions.push_back(cur);
        for (const Action& action : target.actions) {
            cur = ActionSemantics::compute_effect(cur, action).agent_to;
            target.positions.push_back(cur);
        }

        reservations.reserve_path(plan.agent_plan);
        if (task.box_id >= 'A' && task.box_id <= 'Z' && !plan.box_trajectory.empty()) {
            reservations.reserve_box_path(task.box_id, plan.box_trajectory, 0);
        }
        replay_plan_into_state(simulated, task.agent_id, plan.agent_plan);
    }
    return candidate;
}

} // namespace

LNSRepairer::Result LNSRepairer::repair_wave(const Level& level,
                                             const State& initial_state,
                                             const std::vector<Task>& tasks,
                                             const std::vector<AgentPlan>& incumbent,
                                             const PlanningDeadline& deadline,
                                             int max_iterations) const {
    Result result;
    const int num_agents = initial_state.num_agents();
    result.agent_plans = incumbent;
    result.agent_plans.resize(static_cast<std::size_t>(num_agents));
    result.plan = uncompressed_plan_from_agent_plans(result.agent_plans, num_agents);
    result.conflicts_before = count_conflicts(result.agent_plans, initial_state);
    result.conflicts_after = result.conflicts_before;
    if (result.conflicts_before == 0) {
        result.conflict_free = true;
        return result;
    }

    std::vector<AgentPlan> best = result.agent_plans;
    int best_conflicts = result.conflicts_before;
    const int solved_goals_before = WaveProgressGuard::goals_completed(level, initial_state);
    PlanConflictRepairer wait_repair;

    for (int iteration = 0; iteration < max_iterations && !deadline.expired(); ++iteration) {
        result.iterations = iteration + 1;
        const Conflict conflict = ConflictDetector::findFirstConflict(best, initial_state);
        if (!conflict.valid()) break;

        std::vector<int> selected = select_neighborhood(tasks, conflict, num_agents);
        if (selected.empty()) break;

        std::vector<AgentPlan> candidate = replan_neighborhood(level, initial_state, tasks, best, conflict, selected, deadline);
        PlanConflictRepairer::Result smoothed = wait_repair.repair(level, initial_state, candidate, 128, 64);
        if (!smoothed.agent_plans.empty()) candidate = smoothed.agent_plans;

        const int candidate_conflicts = count_conflicts(candidate, initial_state);
        const Plan candidate_plan = uncompressed_plan_from_agent_plans(candidate, num_agents);
        const WaveProgressDecision progress = WaveProgressGuard{}.assess(level, initial_state, candidate_plan, solved_goals_before);
        const bool preserves_solved_goals = progress.goals_after >= solved_goals_before;
        if (candidate_conflicts < best_conflicts || (preserves_solved_goals && candidate_conflicts == 0)) {
            best = std::move(candidate);
            best_conflicts = candidate_conflicts;
            result.changed = true;
            if (best_conflicts == 0) break;
        } else {
            break;
        }
    }

    result.agent_plans = best;
    result.plan = uncompressed_plan_from_agent_plans(best, num_agents);
    result.conflicts_after = best_conflicts;
    result.conflict_free = best_conflicts == 0;
    return result;
}
