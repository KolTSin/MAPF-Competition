#include "hospital/BoxTransportPlanner.hpp"

#include "actions/ActionSemantics.hpp"

#include <array>
#include <queue>
#include <unordered_map>

namespace {
struct Key {
    Position agent;
    Position box;
    int time;
    bool operator==(const Key& o) const noexcept { return agent == o.agent && box == o.box && time == o.time; }
};
struct KeyHash {
    std::size_t operator()(const Key& k) const noexcept {
        return (k.agent.row * 73856093u) ^ (k.agent.col * 19349663u) ^ (k.box.row * 83492791u) ^ (k.box.col * 2654435761u) ^ (k.time * 97531u);
    }
};

struct Node {
    Position agent;
    Position box;
    int g;
    int h;
    int time;
    std::vector<Action> actions;
    std::vector<Position> agent_traj;
    std::vector<Position> box_traj;
};

int manhattan(const Position& a, const Position& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

bool is_free_cell(const Level& level, const State& state, const Position& p, const Position& active_box) {
    if (!level.in_bounds(p.row, p.col) || level.is_wall(p.row, p.col)) return false;
    const char b = state.box_at(p.row, p.col);
    if (b != '\0' && !(p == active_box)) return false;
    return true;
}

bool validate_replay(const Level& level, const State& state, const Task& task, TaskPlan& out) {
    Position agent = state.agent_positions[task.agent_id];
    Position box = task.box_pos;

    if (out.agent_trajectory.empty() || out.box_trajectory.empty()) {
        out.success = false;
        out.failure_reason = "invalid_empty_trajectory";
        return false;
    }
    if (out.agent_trajectory.front() != agent || out.box_trajectory.front() != box) {
        out.success = false;
        out.failure_reason = "invalid_start_trajectory";
        return false;
    }
    if (out.primitive_actions.size() + 1 != out.agent_trajectory.size() ||
        out.primitive_actions.size() + 1 != out.box_trajectory.size()) {
        out.success = false;
        out.failure_reason = "invalid_trajectory_length_mismatch";
        return false;
    }

    for (std::size_t i = 0; i < out.primitive_actions.size(); ++i) {
        const Action action = out.primitive_actions[i];
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
            if (state.box_at(eff.box_to.row, eff.box_to.col) != '\0' && eff.box_to != box) {
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
            if (state.box_at(eff.agent_to.row, eff.agent_to.col) != '\0' && eff.agent_to != box) {
                out.success = false;
                out.failure_reason = "invalid_pull_agent_into_static_box";
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

        if (out.agent_trajectory[i + 1] != agent || out.box_trajectory[i + 1] != box) {
            out.success = false;
            out.failure_reason = "invalid_replay_trajectory_mismatch";
            return false;
        }
    }

    return true;
}

constexpr std::array<Direction, 4> kDirs{Direction::North, Direction::South, Direction::East, Direction::West};
}

TaskPlan BoxTransportPlanner::plan(const Level& level, const State& state, const Task& task) const {
    TaskPlan out;
    out.task_id = task.task_id;
    out.task_type = task.type;
    out.agent_id = task.agent_id;

    if (task.agent_id < 0 || task.agent_id >= state.num_agents()) {
        out.failure_reason = "invalid_agent";
        return out;
    }

    auto cmp = [](const Node& a, const Node& b) { return (a.g + a.h) > (b.g + b.h); };
    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open(cmp);
    std::unordered_map<Key, int, KeyHash> best;

    Node start;
    start.agent = state.agent_positions[task.agent_id];
    start.box = task.box_pos;
    start.g = 0;
    start.h = manhattan(start.box, task.goal_pos);
    start.time = 0;
    start.agent_traj.push_back(start.agent);
    start.box_traj.push_back(start.box);
    open.push(start);
    best[Key{start.agent, start.box, 0}] = 0;

    const int max_expansions = level.rows * level.cols * 200;
    int expansions = 0;

    while (!open.empty() && expansions++ < max_expansions) {
        Node cur = open.top();
        open.pop();

        if (cur.box == task.goal_pos) {
            out.success = true;
            out.primitive_actions = cur.actions;
            out.agent_trajectory = cur.agent_traj;
            out.box_trajectory = cur.box_traj;
            if (!validate_replay(level, state, task, out)) {
                return out;
            }
            return out;
        }

        for (Direction d : kDirs) {
            Position next_agent{cur.agent.row + drow(d), cur.agent.col + dcol(d)};
            if (!is_free_cell(level, state, next_agent, cur.box)) continue;

            Node nxt = cur;
            nxt.g = cur.g + 1;
            nxt.time = cur.time + 1;

            if (next_agent == cur.box) {
                Position next_box{cur.box.row + drow(d), cur.box.col + dcol(d)};
                if (!is_free_cell(level, state, next_box, cur.box)) continue;
                nxt.agent = next_agent;
                nxt.box = next_box;
                nxt.actions.push_back(Action::push(d, d));
            } else {
                nxt.agent = next_agent;
                nxt.actions.push_back(Action::move(d));
            }

            nxt.h = manhattan(nxt.box, task.goal_pos);
            nxt.agent_traj.push_back(nxt.agent);
            nxt.box_traj.push_back(nxt.box);

            Key k{nxt.agent, nxt.box, nxt.time};
            auto it = best.find(k);
            if (it != best.end() && it->second <= nxt.g) continue;
            best[k] = nxt.g;
            open.push(std::move(nxt));
        }

        for (Direction move_dir : kDirs) {
            Position pulled_from{cur.agent.row - drow(move_dir), cur.agent.col - dcol(move_dir)};
            if (!(pulled_from == cur.box)) continue;

            Position next_agent{cur.agent.row + drow(move_dir), cur.agent.col + dcol(move_dir)};
            if (!is_free_cell(level, state, next_agent, cur.box)) continue;

            Node nxt = cur;
            nxt.g = cur.g + 1;
            nxt.time = cur.time + 1;
            nxt.agent = next_agent;
            nxt.box = cur.agent;
            nxt.actions.push_back(Action::pull(move_dir, Direction::North)); // placeholder, set below

            Direction box_dir = Direction::North;
            if (pulled_from.row == cur.agent.row - 1 && pulled_from.col == cur.agent.col) box_dir = Direction::North;
            else if (pulled_from.row == cur.agent.row + 1 && pulled_from.col == cur.agent.col) box_dir = Direction::South;
            else if (pulled_from.row == cur.agent.row && pulled_from.col == cur.agent.col + 1) box_dir = Direction::East;
            else if (pulled_from.row == cur.agent.row && pulled_from.col == cur.agent.col - 1) box_dir = Direction::West;
            nxt.actions.back() = Action::pull(move_dir, box_dir);

            nxt.h = manhattan(nxt.box, task.goal_pos);
            nxt.agent_traj.push_back(nxt.agent);
            nxt.box_traj.push_back(nxt.box);

            Key k{nxt.agent, nxt.box, nxt.time};
            auto it = best.find(k);
            if (it != best.end() && it->second <= nxt.g) continue;
            best[k] = nxt.g;
            open.push(std::move(nxt));
        }
    }

    out.failure_reason = "no_path_for_single_box";
    return out;
}
