#include "hospital/BoxTransportPlanner.hpp"

#include "actions/ActionApplicator.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>

namespace {
struct PrevCell {
    Position prev{-1, -1};
    bool has_prev{false};
};

struct BoxSearchNode {
    Position box;
    Position agent;
};

struct BoxSearchParent {
    int prev_key{-1};
    bool has_prev{false};
    Position box{};
};

constexpr int DR[4] = {-1, 1, 0, 0};
constexpr int DC[4] = {0, 0, -1, 1};
}

BoxTransportPlanner::BoxTransportPlanner(const MapAnalysis& analysis)
    : analysis_(analysis) {}

std::vector<Action> BoxTransportPlanner::plan_for_task(const Level& level, State& state, const HospitalTask& task) const {
    if (task.type != HospitalTaskType::RelocateBox && task.type != HospitalTaskType::TransportBox) {
        return {};
    }

    std::optional<Position> box_pos_opt = find_box(state, task.box_symbol);
    if (!box_pos_opt.has_value()) {
        return {};
    }

    Position box_pos = *box_pos_opt;
    if (box_pos == task.destination) {
        return {};
    }

    std::vector<Action> total_actions;
    std::vector<Position> route = shortest_box_path(
        level,
        state,
        task.agent_id,
        state.agent_positions[task.agent_id],
        box_pos,
        task.destination);
    if (route.size() < 2) {
        return {};
    }

    for (std::size_t i = 0; i + 1 < route.size(); ++i) {
        const Position current_box = route[i];
        const Position next_box = route[i + 1];

        const int dr = next_box.row - current_box.row;
        const int dc = next_box.col - current_box.col;
        const std::optional<Direction> move_dir = direction_from_delta(dr, dc);
        if (!move_dir.has_value()) {
            break;
        }

        const Position setup_cell{current_box.row - dr, current_box.col - dc};
        if (!level.in_bounds(setup_cell.row, setup_cell.col) || level.is_wall(setup_cell.row, setup_cell.col)) {
            break;
        }

        std::vector<Action> walk_actions = shortest_agent_walk(
            level,
            state,
            task.agent_id,
            state.agent_positions[task.agent_id],
            setup_cell);

        if (walk_actions.empty() && state.agent_positions[task.agent_id] != setup_cell) {
            break;
        }

        apply_actions(level, state, task.agent_id, walk_actions);
        total_actions.insert(total_actions.end(), walk_actions.begin(), walk_actions.end());

        const Action push = Action::push(*move_dir, *move_dir);
        if (!ActionApplicator::is_applicable(level, state, task.agent_id, push)) {
            break;
        }

        state = ActionApplicator::apply(level, state, task.agent_id, push);
        total_actions.push_back(push);
    }

    return total_actions;
}

std::optional<Direction> BoxTransportPlanner::direction_from_delta(const int dr, const int dc) {
    if (dr == -1 && dc == 0) return Direction::North;
    if (dr == 1 && dc == 0) return Direction::South;
    if (dr == 0 && dc == 1) return Direction::East;
    if (dr == 0 && dc == -1) return Direction::West;
    return std::nullopt;
}

Position BoxTransportPlanner::add(const Position& p, const Direction dir) {
    return Position{p.row + drow(dir), p.col + dcol(dir)};
}

int BoxTransportPlanner::manhattan(const Position& a, const Position& b) noexcept {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

std::optional<Position> BoxTransportPlanner::find_box(const State& state, const char box_symbol) {
    for (int row = 0; row < state.rows; ++row) {
        for (int col = 0; col < state.cols; ++col) {
            if (state.box_at(row, col) == box_symbol) {
                return Position{row, col};
            }
        }
    }
    return std::nullopt;
}

std::vector<Position> BoxTransportPlanner::shortest_box_path(
    const Level& level,
    const State& state,
    const int agent,
    const Position& agent_start,
    const Position& box_start,
    const Position& box_goal) const {
    const int grid_cells = state.rows * state.cols;
    const char moving_box_symbol = state.box_at(box_start.row, box_start.col);

    auto projected_state = [&](const Position& box_pos) {
        State projected = state;
        projected.set_box(box_start.row, box_start.col, '\0');
        projected.set_box(box_pos.row, box_pos.col, moving_box_symbol);
        return projected;
    };
    auto key_for = [&](const Position& box, const Position& agent_pos) {
        return state.index(box.row, box.col) * grid_cells + state.index(agent_pos.row, agent_pos.col);
    };
    auto decode = [&](int key) -> BoxSearchNode {
        int box_flat = key / grid_cells;
        int agent_flat = key % grid_cells;
        return BoxSearchNode{
            Position{box_flat / state.cols, box_flat % state.cols},
            Position{agent_flat / state.cols, agent_flat % state.cols}
        };
    };

    std::queue<int> q;
    std::unordered_map<int, BoxSearchParent> parents;
    const int start_key = key_for(box_start, agent_start);
    q.push(start_key);
    parents[start_key] = BoxSearchParent{-1, true, box_start};

    int goal_key = -1;
    while (!q.empty()) {
        int current_key = q.front();
        q.pop();

        const BoxSearchNode node = decode(current_key);
        if (node.box == box_goal) {
            goal_key = current_key;
            break;
        }

        const State node_state = projected_state(node.box);

        for (const Position& nxt_box : analysis_.neighbors(node.box)) {
            if (node_state.has_box(nxt_box.row, nxt_box.col) && nxt_box != node.box) {
                continue;
            }

            const int dr = nxt_box.row - node.box.row;
            const int dc = nxt_box.col - node.box.col;
            const Position setup{node.box.row - dr, node.box.col - dc};
            if (!level.in_bounds(setup.row, setup.col) || level.is_wall(setup.row, setup.col)) {
                continue;
            }

            std::vector<Action> setup_walk = shortest_agent_walk(level, node_state, agent, node.agent, setup);
            if (setup_walk.empty() && node.agent != setup) {
                continue;
            }

            const Position nxt_agent = node.box;
            const int nxt_key = key_for(nxt_box, nxt_agent);
            if (parents.contains(nxt_key)) {
                continue;
            }

            parents[nxt_key] = BoxSearchParent{current_key, true, nxt_box};
            q.push(nxt_key);
        }
    }

    if (goal_key < 0) {
        return {};
    }

    std::vector<Position> path;
    int key = goal_key;
    while (key != start_key) {
        const BoxSearchNode node = decode(key);
        path.push_back(node.box);
        key = parents[key].prev_key;
    }
    path.push_back(box_start);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Action> BoxTransportPlanner::shortest_agent_walk(
    const Level& level,
    const State& state,
    const int agent,
    const Position& from,
    const Position& to) const {
    if (from == to) {
        return {};
    }

    std::queue<Position> q;
    std::vector<PrevCell> prev(static_cast<std::size_t>(state.rows * state.cols));

    auto idx = [&](const Position& p) {
        return state.index(p.row, p.col);
    };

    q.push(from);
    prev[idx(from)].has_prev = true;
    prev[idx(from)].prev = from;

    while (!q.empty()) {
        const Position cur = q.front();
        q.pop();

        if (cur == to) {
            break;
        }

        for (const Position& nxt : analysis_.neighbors(cur)) {
            if (state.has_box(nxt.row, nxt.col)) {
                continue;
            }

            if (prev[idx(nxt)].has_prev) {
                continue;
            }

            prev[idx(nxt)].has_prev = true;
            prev[idx(nxt)].prev = cur;
            q.push(nxt);
        }
    }

    if (!prev[idx(to)].has_prev) {
        return {};
    }

    std::vector<Position> path;
    Position cur = to;
    while (cur != from) {
        path.push_back(cur);
        cur = prev[idx(cur)].prev;
    }
    path.push_back(from);
    std::reverse(path.begin(), path.end());

    std::vector<Action> actions;
    actions.reserve(path.size());
    State simulated = state;
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const int dr = path[i + 1].row - path[i].row;
        const int dc = path[i + 1].col - path[i].col;
        const std::optional<Direction> dir = direction_from_delta(dr, dc);
        if (!dir.has_value()) {
            return {};
        }

        const Action move = Action::move(*dir);
        if (!ActionApplicator::is_applicable(level, simulated, agent, move)) {
            return {};
        }

        actions.push_back(move);
        simulated = ActionApplicator::apply(level, simulated, agent, move);
    }

    return actions;
}

void BoxTransportPlanner::apply_actions(const Level& level, State& state, const int agent, const std::vector<Action>& actions) {
    for (const Action& action : actions) {
        state = ActionApplicator::apply(level, state, agent, action);
    }
}
