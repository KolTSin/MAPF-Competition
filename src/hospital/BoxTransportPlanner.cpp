#include "hospital/BoxTransportPlanner.hpp"

#include "actions/ActionSemantics.hpp"
#include "search/SuccessorGenerator.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

struct Key {
    Position agent;
    Position box;
    int time;
    bool operator==(const Key& o) const noexcept { return agent == o.agent && box == o.box && time == o.time; }
};

struct KeyHash {
    std::size_t operator()(const Key& k) const noexcept {
        return (static_cast<std::size_t>(k.agent.row) * 73856093u) ^
               (static_cast<std::size_t>(k.agent.col) * 19349663u) ^
               (static_cast<std::size_t>(k.box.row) * 83492791u) ^
               (static_cast<std::size_t>(k.box.col) * 2654435761u) ^
               (static_cast<std::size_t>(k.time) * 97531u);
    }
};

struct Node {
    Position agent;
    Position box;
    int g{0};
    int h{0};
    int time{0};
    int parent{-1};
    Action action{Action::noop()};

    [[nodiscard]] int f() const noexcept { return g + h; }
};

int manhattan(const Position& a, const Position& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

int box_transport_heuristic(const Position& agent, const Position& box, const Position& goal) {
    return manhattan(agent, box) + manhattan(box, goal);
}

void set_failure(TaskPlan& out,
                 TaskFailureCause cause,
                 std::string reason,
                 const TaskFailureInfo& detail = {}) {
    out.success = false;
    out.failure_reason = std::move(reason);
    out.failure_info = detail;
    out.failure_info.cause = cause;
}

bool wall_connected(const Level& level, Position start, Position goal) {
    if (!level.in_bounds(start.row, start.col) || !level.in_bounds(goal.row, goal.col)) return false;
    if (level.is_wall(start.row, start.col) || level.is_wall(goal.row, goal.col)) return false;
    std::vector<bool> seen(static_cast<std::size_t>(level.rows * level.cols), false);
    std::deque<Position> q;
    seen[static_cast<std::size_t>(level.index(start.row, start.col))] = true;
    q.push_back(start);
    constexpr int dr[4] = {-1, 1, 0, 0};
    constexpr int dc[4] = {0, 0, -1, 1};
    while (!q.empty()) {
        const Position cur = q.front();
        q.pop_front();
        if (cur == goal) return true;
        for (int i = 0; i < 4; ++i) {
            const Position next{cur.row + dr[i], cur.col + dc[i]};
            if (!level.in_bounds(next.row, next.col) || level.is_wall(next.row, next.col)) continue;
            const std::size_t idx = static_cast<std::size_t>(level.index(next.row, next.col));
            if (seen[idx]) continue;
            seen[idx] = true;
            q.push_back(next);
        }
    }
    return false;
}

std::vector<bool> agent_reachable_without_moving_boxes(const Level& level,
                                                       const State& state,
                                                       int agent_id) {
    std::vector<bool> seen(static_cast<std::size_t>(level.rows * level.cols), false);
    if (agent_id < 0 || agent_id >= state.num_agents()) return seen;

    const Position start = state.agent_positions[agent_id];
    if (!level.in_bounds(start.row, start.col) || level.is_wall(start.row, start.col)) return seen;
    std::deque<Position> q;
    seen[static_cast<std::size_t>(level.index(start.row, start.col))] = true;
    q.push_back(start);

    constexpr int dr[4] = {-1, 1, 0, 0};
    constexpr int dc[4] = {0, 0, -1, 1};
    while (!q.empty()) {
        const Position cur = q.front();
        q.pop_front();
        for (int i = 0; i < 4; ++i) {
            const Position next{cur.row + dr[i], cur.col + dc[i]};
            if (!level.in_bounds(next.row, next.col) || level.is_wall(next.row, next.col)) continue;
            if (state.has_box(next.row, next.col)) continue;
            const std::size_t idx = static_cast<std::size_t>(level.index(next.row, next.col));
            if (seen[idx]) continue;
            seen[idx] = true;
            q.push_back(next);
        }
    }
    return seen;
}

Position nearest_reachable_push_side(const Level& level,
                                     const std::vector<bool>& reachable,
                                     Position box) {
    Position best{-1, -1};
    int best_dist = std::numeric_limits<int>::max();
    constexpr int dr[4] = {-1, 1, 0, 0};
    constexpr int dc[4] = {0, 0, -1, 1};
    for (int i = 0; i < 4; ++i) {
        const Position side{box.row + dr[i], box.col + dc[i]};
        if (!level.in_bounds(side.row, side.col)) continue;
        if (!reachable[static_cast<std::size_t>(level.index(side.row, side.col))]) continue;
        const int dist = manhattan(side, box);
        if (dist < best_dist) {
            best_dist = dist;
            best = side;
        }
    }
    return best;
}

bool validate_replay(const Level& level, const State& state, const Task& task, TaskPlan& out) {
    Position agent = state.agent_positions[task.agent_id];
    Position box = task.box_pos;

    if (out.agent_plan.positions.empty() || out.box_trajectory.empty()) {
        set_failure(out, TaskFailureCause::NoPathForSingleBox, "invalid_empty_trajectory");
        return false;
    }
    if (out.agent_plan.positions.front() != agent || out.box_trajectory.front() != box) {
        set_failure(out, TaskFailureCause::NoPathForSingleBox, "invalid_start_trajectory");
        return false;
    }
    if (out.agent_plan.actions.size() + 1 != out.agent_plan.positions.size() ||
        out.agent_plan.actions.size() + 1 != out.box_trajectory.size()) {
        set_failure(out, TaskFailureCause::NoPathForSingleBox, "invalid_trajectory_length_mismatch");
        return false;
    }

    for (std::size_t i = 0; i < out.agent_plan.actions.size(); ++i) {
        const Action action = out.agent_plan.actions[i];
        const ActionEffect eff = ActionSemantics::compute_effect(agent, action);

        if (!level.in_bounds(eff.agent_to.row, eff.agent_to.col) || level.is_wall(eff.agent_to.row, eff.agent_to.col)) {
            set_failure(out, TaskFailureCause::NoPathForSingleBox, "invalid_agent_step_wall_or_oob");
            return false;
        }

        if (action.type == ActionType::Push) {
            if (eff.box_from != box) {
                set_failure(out, TaskFailureCause::NoPathForSingleBox, "invalid_push_without_adjacent_box");
                return false;
            }
            if (!level.in_bounds(eff.box_to.row, eff.box_to.col) || level.is_wall(eff.box_to.row, eff.box_to.col)) {
                set_failure(out, TaskFailureCause::NoPathForSingleBox, "invalid_push_box_wall_or_oob");
                return false;
            }
            if (state.box_at(eff.box_to.row, eff.box_to.col) != '\0' && eff.box_to != box && eff.box_to != task.box_pos) {
                set_failure(out, TaskFailureCause::NoPathForSingleBox, "invalid_push_into_static_box");
                return false;
            }
            box = eff.box_to;
        } else if (action.type == ActionType::Pull) {
            if (eff.box_from != box) {
                set_failure(out, TaskFailureCause::NoPathForSingleBox, "invalid_pull_without_adjacent_box");
                return false;
            }
            if (!level.in_bounds(eff.agent_to.row, eff.agent_to.col) || level.is_wall(eff.agent_to.row, eff.agent_to.col)) {
                set_failure(out, TaskFailureCause::NoPathForSingleBox, "invalid_pull_agent_wall_or_oob");
                return false;
            }
            box = eff.box_to;
        } else if (action.type == ActionType::Move) {
            if (eff.agent_to == box) {
                set_failure(out, TaskFailureCause::NoPathForSingleBox, "invalid_move_into_box");
                return false;
            }
        }

        agent = eff.agent_to;

        if (out.agent_plan.positions[i + 1] != agent || out.box_trajectory[i + 1] != box) {
            set_failure(out, TaskFailureCause::NoPathForSingleBox, "invalid_replay_trajectory_mismatch");
            return false;
        }
    }

    return true;
}

TaskPlan reconstruct_task_plan(const Level& level,
                               const State& state,
                               const Task& task,
                               const std::vector<Node>& nodes,
                               int goal_index) {
    TaskPlan out;
    out.task_id = task.task_id;
    out.task_type = task.type;
    out.agent_id = task.agent_id;
    out.agent_plan.agent = task.agent_id;

    std::vector<Action> reversed_actions;
    std::vector<Position> reversed_agents;
    std::vector<Position> reversed_boxes;

    int cur = goal_index;
    reversed_agents.push_back(nodes[cur].agent);
    reversed_boxes.push_back(nodes[cur].box);
    while (nodes[cur].parent != -1) {
        reversed_actions.push_back(nodes[cur].action);
        cur = nodes[cur].parent;
        reversed_agents.push_back(nodes[cur].agent);
        reversed_boxes.push_back(nodes[cur].box);
    }

    std::reverse(reversed_actions.begin(), reversed_actions.end());
    std::reverse(reversed_agents.begin(), reversed_agents.end());
    std::reverse(reversed_boxes.begin(), reversed_boxes.end());

    out.success = true;
    out.agent_plan.actions = std::move(reversed_actions);
    out.agent_plan.positions = std::move(reversed_agents);
    out.box_trajectory = std::move(reversed_boxes);
    validate_replay(level, state, task, out);
    return out;
}

} // namespace

TaskPlan BoxTransportPlanner::plan(const Level& level, const State& state, const Task& task) const {
    ReservationTable empty_reservations;
    return plan(level, state, task, empty_reservations, 0);
}

TaskPlan BoxTransportPlanner::plan(const Level& level,
                                   const State& state,
                                   const Task& task,
                                   const ReservationTable& reservations,
                                   int start_time) const {
    PlanningDeadline no_deadline;
    return plan(level, state, task, reservations, start_time, no_deadline);
}

TaskPlan BoxTransportPlanner::plan(const Level& level,
                                   const State& state,
                                   const Task& task,
                                   const ReservationTable& reservations,
                                   int start_time,
                                   const PlanningDeadline& deadline) const {
    TaskPlan out;
    out.task_id = task.task_id;
    out.task_type = task.type;
    out.agent_id = task.agent_id;
    out.agent_plan.agent = task.agent_id;

    if (deadline.expired()) {
        set_failure(out, TaskFailureCause::ExpansionLimitReached, "planning_deadline_expired");
        return out;
    }
    if (task.agent_id < 0 || task.agent_id >= state.num_agents()) {
        set_failure(out, TaskFailureCause::InvalidAgent, "invalid_agent");
        return out;
    }
    if (task.box_id < 'A' || task.box_id > 'Z') {
        set_failure(out, TaskFailureCause::InvalidBox, "invalid_box");
        return out;
    }
    if (!reservations.can_occupy_agent(task.agent_id, state.agent_positions[task.agent_id], start_time) ||
        !reservations.can_occupy_box(task.box_id, task.box_pos, start_time)) {
        TaskFailureInfo detail;
        detail.blocking_position = !reservations.can_occupy_agent(task.agent_id, state.agent_positions[task.agent_id], start_time)
            ? state.agent_positions[task.agent_id]
            : task.box_pos;
        set_failure(out, TaskFailureCause::ReservedStartCell, "reserved_start_cell", detail);
        return out;
    }
    if (!level.in_bounds(task.goal_pos.row, task.goal_pos.col) || level.is_wall(task.goal_pos.row, task.goal_pos.col)) {
        TaskFailureInfo detail;
        detail.blocking_position = task.goal_pos;
        detail.blocking_object = '#';
        set_failure(out, TaskFailureCause::BoxDestinationBlockedByWall, "box_destination_blocked_by_wall", detail);
        return out;
    }
    const char goal_box = state.box_at(task.goal_pos.row, task.goal_pos.col);
    if (goal_box != '\0' && task.goal_pos != task.box_pos) {
        TaskFailureInfo detail;
        detail.blocking_position = task.goal_pos;
        detail.blocking_object = goal_box;
        set_failure(out, TaskFailureCause::BoxDestinationBlockedByBox, "box_destination_blocked_by_box", detail);
        return out;
    }
    if (!wall_connected(level, task.box_pos, task.goal_pos)) {
        TaskFailureInfo detail;
        detail.nearest_box_to_goal = task.box_pos;
        set_failure(out, TaskFailureCause::StaticComponentUnreachable, "static_component_unreachable", detail);
        return out;
    }

    const std::vector<bool> initial_agent_reachability =
        agent_reachable_without_moving_boxes(level, state, task.agent_id);
    const Position initial_push_side = nearest_reachable_push_side(level, initial_agent_reachability, task.box_pos);
    if (task.box_pos != task.goal_pos && initial_push_side.row < 0) {
        TaskFailureInfo detail;
        detail.nearest_box_to_goal = task.box_pos;
        set_failure(out,
                    TaskFailureCause::AgentCannotReachRequiredPushSide,
                    "agent_cannot_reach_required_push_side",
                    detail);
        return out;
    }

    std::vector<Node> nodes;
    nodes.reserve(4096);

    auto cmp = [&nodes](int a, int b) {
        const Node& lhs = nodes[a];
        const Node& rhs = nodes[b];
        if (lhs.f() != rhs.f()) return lhs.f() > rhs.f();
        if (lhs.h != rhs.h) return lhs.h > rhs.h;
        return lhs.g < rhs.g;
    };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);
    std::unordered_map<Key, int, KeyHash> best;
    best.reserve(4096);
    const bool time_aware_closed_set = !reservations.empty();

    const Position start_agent = state.agent_positions[task.agent_id];
    nodes.push_back(Node{start_agent, task.box_pos, 0, box_transport_heuristic(start_agent, task.box_pos, task.goal_pos), 0, -1, Action::noop()});
    open.push(0);
    best[Key{start_agent, task.box_pos, time_aware_closed_set ? 0 : 0}] = 0;

    const int max_expansions = std::max(1000, level.rows * level.cols * 250);
    const int max_time = std::max(256, level.rows * level.cols * 20);
    if (task.box_pos != task.goal_pos && !reservations.empty()) {
        bool destination_reserved_for_entire_horizon = true;
        for (int dt = 1; dt <= max_time; ++dt) {
            if (reservations.can_occupy_box(task.box_id, task.goal_pos, start_time + dt)) {
                destination_reserved_for_entire_horizon = false;
                break;
            }
        }
        if (destination_reserved_for_entire_horizon) {
            TaskFailureInfo detail;
            detail.blocking_position = task.goal_pos;
            detail.max_time = max_time;
            set_failure(out, TaskFailureCause::BoxDestinationReserved, "box_destination_reserved", detail);
            return out;
        }
    }
    int expansions = 0;
    std::vector<BoxTransportSuccessor> successors;
    successors.reserve(29);

    int best_box_distance = std::numeric_limits<int>::max();
    Position best_box = task.box_pos;
    Position best_push_side = initial_push_side;
    bool hit_time_horizon = false;
    bool generated_any_successor = false;

    while (!open.empty() && expansions++ < max_expansions) {
        if (deadline.expired()) {
            set_failure(out, TaskFailureCause::ExpansionLimitReached, "planning_deadline_expired");
            return out;
        }
        const int current_index = open.top();
        open.pop();
        const Node current = nodes[current_index];

        const int box_distance = manhattan(current.box, task.goal_pos);
        if (box_distance < best_box_distance) {
            best_box_distance = box_distance;
            best_box = current.box;
            const std::vector<bool> reachable = agent_reachable_without_moving_boxes(level, state, task.agent_id);
            const Position side = nearest_reachable_push_side(level, reachable, current.box);
            if (side.row >= 0) best_push_side = side;
        }

        const Key current_key{current.agent, current.box, time_aware_closed_set ? current.time : 0};
        const auto it_best = best.find(current_key);
        if (it_best != best.end() && current.g > it_best->second) continue;

        if (current.box == task.goal_pos) {
            return reconstruct_task_plan(level, state, task, nodes, current_index);
        }
        if (current.time >= max_time) {
            hit_time_horizon = true;
            continue;
        }

        SuccessorGenerator::expand_box_transport(level,
                                                 state,
                                                 task.agent_id,
                                                 task.box_id,
                                                 task.box_pos,
                                                 BoxTransportSearchState{current.agent, current.box, current.time},
                                                 start_time,
                                                 reservations,
                                                 successors);

        generated_any_successor = generated_any_successor || !successors.empty();

        for (const BoxTransportSuccessor& succ : successors) {
            Node next;
            next.agent = succ.next.agent;
            next.box = succ.next.box;
            next.g = current.g + 1;
            next.h = box_transport_heuristic(next.agent, next.box, task.goal_pos);
            next.time = succ.next.time;
            next.parent = current_index;
            next.action = succ.action;

            const Key key{next.agent, next.box, time_aware_closed_set ? next.time : 0};
            auto it = best.find(key);
            if (it != best.end() && it->second <= next.g) continue;

            nodes.push_back(next);
            const int next_index = static_cast<int>(nodes.size()) - 1;
            best[key] = next.g;
            open.push(next_index);
        }
    }

    TaskFailureInfo detail;
    detail.nearest_box_to_goal = best_box;
    detail.nearest_reachable_push_side = best_push_side;
    detail.expansions = expansions;
    detail.max_expansions = max_expansions;
    detail.max_time = max_time;
    if (expansions >= max_expansions && !open.empty()) {
        set_failure(out, TaskFailureCause::ExpansionLimitReached, "box_transport_expansion_limit_reached", detail);
    } else if (hit_time_horizon) {
        set_failure(out, TaskFailureCause::TimeHorizonReached, "box_transport_time_horizon_reached", detail);
    } else if (!generated_any_successor) {
        set_failure(out, TaskFailureCause::NoLegalBoxTransportSuccessors, "no_legal_box_transport_successors", detail);
    } else {
        set_failure(out, TaskFailureCause::NoPathForSingleBox, "no_path_for_single_box", detail);
    }
    return out;
}
