#include "hospital/BoxTransportPlanner.hpp"

#include "actions/ActionApplicator.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>

namespace {
struct BoxSearchNode {
    Position box;
    Position agent;
};

struct BoxSearchParent {
    int prev_key{-1};
    bool has_prev{false};
    Action action{Action::noop()};
};

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

    std::vector<Action> total_actions = shortest_box_action_plan(
        level,
        state,
        task.agent_id,
        state.agent_positions[task.agent_id],
        box_pos,
        task.destination);
    if (total_actions.empty()) {
        return {};
    }
    for (const Action& action : total_actions) {
        if (!ActionApplicator::is_applicable(level, state, task.agent_id, action)) {
            return {};
        }
        state = ActionApplicator::apply(level, state, task.agent_id, action);
    }

    return total_actions;
}

std::vector<Action> BoxTransportPlanner::shortest_box_action_plan(
    const Level& level,
    const State& state,
    const int agent,
    const Position& agent_start,
    const Position& box_start,
    const Position& box_goal) const {
    const int grid_cells = state.rows * state.cols;
    auto key_for = [&](const Position& box, const Position& agent_pos) {
        return state.index(box.row, box.col) * grid_cells + state.index(agent_pos.row, agent_pos.col);
    };
    auto decode = [&](int key) -> BoxSearchNode {
        const int box_flat = key / grid_cells;
        const int agent_flat = key % grid_cells;
        return BoxSearchNode{
            Position{box_flat / state.cols, box_flat % state.cols},
            Position{agent_flat / state.cols, agent_flat % state.cols}
        };
    };

    const char moving_box = state.box_at(box_start.row, box_start.col);
    auto projected_state = [&](const Position& box_pos, const Position& agent_pos) {
        State projected = state;
        projected.set_box(box_start.row, box_start.col, '\0');
        projected.set_box(box_pos.row, box_pos.col, moving_box);
        projected.agent_positions[agent] = agent_pos;
        return projected;
    };

    std::queue<int> q;
    std::unordered_map<int, BoxSearchParent> parents;
    const int start_key = key_for(box_start, agent_start);
    q.push(start_key);
    parents[start_key] = BoxSearchParent{-1, true, Action::noop()};

    int goal_key = -1;
    while (!q.empty()) {
        const int current_key = q.front();
        q.pop();
        const BoxSearchNode node = decode(current_key);
        if (node.box == box_goal) {
            goal_key = current_key;
            break;
        }

        const State node_state = projected_state(node.box, node.agent);
        for (Direction move_dir : {Direction::North, Direction::South, Direction::East, Direction::West}) {
            const Action move = Action::move(move_dir);
            if (ActionApplicator::is_applicable(level, node_state, agent, move)) {
                const Position next_agent = add(node.agent, move_dir);
                const int next_key = key_for(node.box, next_agent);
                if (!parents.contains(next_key)) {
                    parents[next_key] = BoxSearchParent{current_key, true, move};
                    q.push(next_key);
                }
            }

            const Action push = Action::push(move_dir, move_dir);
            if (ActionApplicator::is_applicable(level, node_state, agent, push)) {
                const Position next_box = add(node.box, move_dir);
                const Position next_agent = node.box;
                const int next_key = key_for(next_box, next_agent);
                if (!parents.contains(next_key)) {
                    parents[next_key] = BoxSearchParent{current_key, true, push};
                    q.push(next_key);
                }
            }

            for (Direction box_dir : {Direction::North, Direction::South, Direction::East, Direction::West}) {
                const Action pull = Action::pull(move_dir, box_dir);
                if (!ActionApplicator::is_applicable(level, node_state, agent, pull)) {
                    continue;
                }

                const Position pulled_box_from{node.agent.row + drow(box_dir), node.agent.col + dcol(box_dir)};
                if (pulled_box_from != node.box) {
                    continue;
                }

                const Position next_agent = add(node.agent, move_dir);
                const Position next_box = node.agent;
                const int next_key = key_for(next_box, next_agent);
                if (!parents.contains(next_key)) {
                    parents[next_key] = BoxSearchParent{current_key, true, pull};
                    q.push(next_key);
                }
            }
        }
    }

    if (goal_key < 0) {
        return {};
    }

    std::vector<Action> actions;
    int key = goal_key;
    while (key != start_key) {
        const BoxSearchParent& parent = parents[key];
        actions.push_back(parent.action);
        key = parent.prev_key;
    }
    std::reverse(actions.begin(), actions.end());
    return actions;
}

Position BoxTransportPlanner::add(const Position& p, const Direction dir) {
    return Position{p.row + drow(dir), p.col + dcol(dir)};
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
