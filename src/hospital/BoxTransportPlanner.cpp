#include "hospital/BoxTransportPlanner.hpp"

#include "actions/ActionSemantics.hpp"
#include "search/SuccessorGenerator.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
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

bool validate_replay(const Level& level, const State& state, const Task& task, TaskPlan& out) {
    Position agent = state.agent_positions[task.agent_id];
    Position box = task.box_pos;

    if (out.agent_plan.positions.empty() || out.box_trajectory.empty()) {
        out.success = false;
        out.failure_reason = "invalid_empty_trajectory";
        return false;
    }
    if (out.agent_plan.positions.front() != agent || out.box_trajectory.front() != box) {
        out.success = false;
        out.failure_reason = "invalid_start_trajectory";
        return false;
    }
    if (out.agent_plan.actions.size() + 1 != out.agent_plan.positions.size() ||
        out.agent_plan.actions.size() + 1 != out.box_trajectory.size()) {
        out.success = false;
        out.failure_reason = "invalid_trajectory_length_mismatch";
        return false;
    }

    for (std::size_t i = 0; i < out.agent_plan.actions.size(); ++i) {
        const Action action = out.agent_plan.actions[i];
        const ActionEffect eff = ActionSemantics::compute_effect(agent, action);

        if (!level.in_bounds(eff.agent_to.row, eff.agent_to.col) || level.is_wall(eff.agent_to.row, eff.agent_to.col)) {
            out.success = false;
            out.failure_reason = "invalid_agent_step_wall_or_oob";
            return false;
        }

        if (action.type == ActionType::Push) {
            if (eff.box_from != box) {
                out.success = false;
                out.failure_reason = "invalid_push_without_adjacent_box";
                return false;
            }
            if (!level.in_bounds(eff.box_to.row, eff.box_to.col) || level.is_wall(eff.box_to.row, eff.box_to.col)) {
                out.success = false;
                out.failure_reason = "invalid_push_box_wall_or_oob";
                return false;
            }
            if (state.box_at(eff.box_to.row, eff.box_to.col) != '\0' && eff.box_to != box && eff.box_to != task.box_pos) {
                out.success = false;
                out.failure_reason = "invalid_push_into_static_box";
                return false;
            }
            box = eff.box_to;
        } else if (action.type == ActionType::Pull) {
            if (eff.box_from != box) {
                out.success = false;
                out.failure_reason = "invalid_pull_without_adjacent_box";
                return false;
            }
            if (!level.in_bounds(eff.agent_to.row, eff.agent_to.col) || level.is_wall(eff.agent_to.row, eff.agent_to.col)) {
                out.success = false;
                out.failure_reason = "invalid_pull_agent_wall_or_oob";
                return false;
            }
            box = eff.box_to;
        } else if (action.type == ActionType::Move) {
            if (eff.agent_to == box) {
                out.success = false;
                out.failure_reason = "invalid_move_into_box";
                return false;
            }
        }

        agent = eff.agent_to;

        if (out.agent_plan.positions[i + 1] != agent || out.box_trajectory[i + 1] != box) {
            out.success = false;
            out.failure_reason = "invalid_replay_trajectory_mismatch";
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
    TaskPlan out;
    out.task_id = task.task_id;
    out.task_type = task.type;
    out.agent_id = task.agent_id;
    out.agent_plan.agent = task.agent_id;

    if (task.agent_id < 0 || task.agent_id >= state.num_agents()) {
        out.failure_reason = "invalid_agent";
        return out;
    }
    if (task.box_id < 'A' || task.box_id > 'Z') {
        out.failure_reason = "invalid_box";
        return out;
    }
    if (!reservations.can_occupy_agent(task.agent_id, state.agent_positions[task.agent_id], start_time) ||
        !reservations.can_occupy_box(task.box_id, task.box_pos, start_time)) {
        out.failure_reason = "reserved_start_cell";
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
    nodes.push_back(Node{start_agent, task.box_pos, 0, manhattan(task.box_pos, task.goal_pos), 0, -1, Action::noop()});
    open.push(0);
    best[Key{start_agent, task.box_pos, time_aware_closed_set ? 0 : 0}] = 0;

    const int max_expansions = std::max(1000, level.rows * level.cols * 250);
    const int max_time = std::max(256, level.rows * level.cols * 20);
    int expansions = 0;
    std::vector<BoxTransportSuccessor> successors;
    successors.reserve(29);

    while (!open.empty() && expansions++ < max_expansions) {
        const int current_index = open.top();
        open.pop();
        const Node current = nodes[current_index];

        const Key current_key{current.agent, current.box, time_aware_closed_set ? current.time : 0};
        const auto it_best = best.find(current_key);
        if (it_best != best.end() && current.g > it_best->second) continue;

        if (current.box == task.goal_pos) {
            return reconstruct_task_plan(level, state, task, nodes, current_index);
        }
        if (current.time >= max_time) continue;

        SuccessorGenerator::expand_box_transport(level,
                                                 state,
                                                 task.agent_id,
                                                 task.box_id,
                                                 task.box_pos,
                                                 BoxTransportSearchState{current.agent, current.box, current.time},
                                                 start_time,
                                                 reservations,
                                                 successors);

        for (const BoxTransportSuccessor& succ : successors) {
            Node next;
            next.agent = succ.next.agent;
            next.box = succ.next.box;
            next.g = current.g + 1;
            next.h = manhattan(next.box, task.goal_pos);
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

    out.failure_reason = "no_path_for_single_box";
    return out;
}
