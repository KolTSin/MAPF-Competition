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
    std::vector<Position> route = shortest_box_path(level, state, box_pos, task.destination);
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
    const Position& from,
    const Position& to) const {
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
            if (state.has_box(nxt.row, nxt.col) && nxt != to) {
                continue;
            }

            const Position support{cur.row - (nxt.row - cur.row), cur.col - (nxt.col - cur.col)};
            if (!level.in_bounds(support.row, support.col) || level.is_wall(support.row, support.col)) {
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
