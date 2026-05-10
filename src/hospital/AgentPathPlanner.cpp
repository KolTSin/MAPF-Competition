#include "hospital/AgentPathPlanner.hpp"

#include "actions/ActionSemantics.hpp"

TaskPlan AgentPathPlanner::plan(const Level& level, const State& state, const Task& task, const ReservationTable& reservations) const {
    return plan(level, state, task, reservations, 0);
}

TaskPlan AgentPathPlanner::plan(const Level& level, const State& state, const Task& task, const ReservationTable& reservations, int start_time) const {
    TaskPlan out;
    out.task_id = task.task_id;
    out.task_type = task.type;
    out.agent_id = task.agent_id;
    out.agent_plan.agent = task.agent_id;

    if (task.agent_id < 0 || task.agent_id >= state.num_agents()) {
        out.failure_reason = "invalid_agent";
        return out;
    }

    Position cur = state.agent_positions[task.agent_id];
    out.agent_plan.positions.push_back(cur);
    out.box_trajectory.push_back(task.box_pos);

    while (cur.row != task.goal_pos.row) {
        Direction d = (cur.row < task.goal_pos.row) ? Direction::South : Direction::North;
        Action a = Action::move(d);
        Position nxt = ActionSemantics::compute_effect(cur, a).agent_to;
        if (!level.in_bounds(nxt.row, nxt.col) || level.is_wall(nxt.row, nxt.col) || reservations.is_cell_reserved(nxt.row, nxt.col, start_time + static_cast<int>(out.agent_plan.actions.size()) + 1, task.agent_id)) {
            out.failure_reason = "no_path_for_agent_reposition";
            return out;
        }
        out.agent_plan.actions.push_back(a);
        cur = nxt;
        out.agent_plan.positions.push_back(cur);
        out.box_trajectory.push_back(task.box_pos);
    }
    while (cur.col != task.goal_pos.col) {
        Direction d = (cur.col < task.goal_pos.col) ? Direction::East : Direction::West;
        Action a = Action::move(d);
        Position nxt = ActionSemantics::compute_effect(cur, a).agent_to;
        if (!level.in_bounds(nxt.row, nxt.col) || level.is_wall(nxt.row, nxt.col) || reservations.is_cell_reserved(nxt.row, nxt.col, start_time + static_cast<int>(out.agent_plan.actions.size()) + 1, task.agent_id)) {
            out.failure_reason = "no_path_for_agent_reposition";
            return out;
        }
        out.agent_plan.actions.push_back(a);
        cur = nxt;
        out.agent_plan.positions.push_back(cur);
        out.box_trajectory.push_back(task.box_pos);
    }

    out.success = true;
    return out;
}
