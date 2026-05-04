#include "hospital/BoxTransportPlanner.hpp"

#include "actions/ActionApplicator.hpp"
#include "actions/ActionLibrary.hpp"
#include "actions/ActionSemantics.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
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

struct BoxInteraction {
    Action action;
    Position setup;
};

Direction opposite(const Direction dir) {
    switch (dir) {
        case Direction::North: return Direction::South;
        case Direction::South: return Direction::North;
        case Direction::East: return Direction::West;
        case Direction::West: return Direction::East;
    }
    return Direction::North;
}

std::optional<Direction> local_direction_from_delta(const int dr, const int dc) {
    if (dr == -1 && dc == 0) return Direction::North;
    if (dr == 1 && dc == 0) return Direction::South;
    if (dr == 0 && dc == 1) return Direction::East;
    if (dr == 0 && dc == -1) return Direction::West;
    return std::nullopt;
}

std::vector<BoxInteraction> interactions_for_box_step(const Position& box_from, const Position& box_to) {
    const int dr = box_to.row - box_from.row;
    const int dc = box_to.col - box_from.col;

    std::vector<BoxInteraction> out;
    const std::optional<Direction> box_move = local_direction_from_delta(dr, dc);
    if (!box_move.has_value()) {
        return out;
    }

    const Direction required_pull_box_dir = *box_move;
    for (const Action& action : ActionLibrary::ALL_ACTIONS) {
        if (action.type == ActionType::Push) {
            if (action.box_dir != *box_move) {
                continue;
            }

            const Position setup{
                box_from.row - drow(action.move_dir),
                box_from.col - dcol(action.move_dir)
            };
            out.push_back(BoxInteraction{action, setup});
        } else if (action.type == ActionType::Pull) {
            if (action.box_dir != required_pull_box_dir) {
                continue;
            }

            out.push_back(BoxInteraction{action, box_to});
        }
    }

    return out;
}

std::optional<Position> find_current_box(const State& state, const char symbol, const Position& preferred) {
    if (state.in_bounds(preferred.row, preferred.col) && state.box_at(preferred.row, preferred.col) == symbol) {
        return preferred;
    }

    for (int row = 0; row < state.rows; ++row) {
        for (int col = 0; col < state.cols; ++col) {
            if (state.box_at(row, col) == symbol) {
                return Position{row, col};
            }
        }
    }

    return std::nullopt;
}

}

BoxTransportPlanner::BoxTransportPlanner(const MapAnalysis& analysis)
    : analysis_(analysis) {}

std::vector<Action> BoxTransportPlanner::plan_for_task(const Level& level, State& state, const Task& task) const {
    if (task.type != TaskType::RelocateBox && task.type != TaskType::TransportBox) {
        return {};
    }

    std::optional<Position> box_pos_opt = find_current_box(state, task.box_symbol, task.source);
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
        task.destination,
        task.box_symbol);
    if (route.size() < 2) {
        std::cerr << "[box-transport] failed route"
                  << " agent=" << task.agent_id
                  << " box=" << task.box_symbol
                  << " from=(" << box_pos.row << "," << box_pos.col << ")"
                  << " to=(" << task.destination.row << "," << task.destination.col << ")"
                  << "\n";
        return {};
    }

    for (std::size_t i = 0; i + 1 < route.size(); ++i) {
        const Position current_box = route[i];
        const Position next_box = route[i + 1];

        bool applied = false;

        const std::vector<BoxInteraction> interactions = interactions_for_box_step(current_box, next_box);
        for (const BoxInteraction& interaction : interactions) {
            if (!level.in_bounds(interaction.setup.row, interaction.setup.col)
                || level.is_wall(interaction.setup.row, interaction.setup.col)) {
                continue;
            }

            std::vector<Action> walk_actions = shortest_agent_walk(
                level,
                state,
                task.agent_id,
                state.agent_positions[task.agent_id],
                interaction.setup);

            if (walk_actions.empty() && state.agent_positions[task.agent_id] != interaction.setup) {
                continue;
            }

            State try_state = state;
            apply_actions(level, try_state, task.agent_id, walk_actions);
            if (!ActionApplicator::is_applicable(level, try_state, task.agent_id, interaction.action)) {
                continue;
            }

            const ActionEffect effect =
                ActionSemantics::compute_effect(try_state.agent_positions[task.agent_id], interaction.action);
            if (!effect.moves_box || effect.box_from != current_box || effect.box_to != next_box) {
                continue;
            }

            apply_actions(level, state, task.agent_id, walk_actions);
            total_actions.insert(total_actions.end(), walk_actions.begin(), walk_actions.end());
            state = ActionApplicator::apply(level, state, task.agent_id, interaction.action);
            total_actions.push_back(interaction.action);
            applied = true;
            break;
        }

        if (!applied) {
            break;
        }
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
    const Position& box_goal,
    const char box_symbol) const {
    int grid_cells = state.rows * state.cols;
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
    auto make_virtual_state = [&](const BoxSearchNode& node) {
        State virtual_state = state;

        virtual_state.set_box(box_start.row, box_start.col, '\0');
        virtual_state.set_box(node.box.row, node.box.col, box_symbol);
        virtual_state.agent_positions[agent] = node.agent;

        return virtual_state;
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

        State virtual_state = make_virtual_state(node);

        for (const Position& nxt_box : analysis_.neighbors(node.box)) {
            if (virtual_state.has_box(nxt_box.row, nxt_box.col) && nxt_box != node.box) {
                continue;
            }

            const std::vector<BoxInteraction> interactions =
                interactions_for_box_step(node.box, nxt_box);

            for (const BoxInteraction& interaction : interactions) {
                if (!level.in_bounds(interaction.setup.row, interaction.setup.col)
                    || level.is_wall(interaction.setup.row, interaction.setup.col)) {
                    continue;
                }

                if (virtual_state.has_box(interaction.setup.row, interaction.setup.col)
                    && interaction.setup != node.box) {
                    continue;
                }

                std::vector<Action> setup_walk =
                    shortest_agent_walk(level, virtual_state, agent, node.agent, interaction.setup);

                if (setup_walk.empty() && node.agent != interaction.setup) {
                    continue;
                }

                State try_state = virtual_state;
                apply_actions(level, try_state, agent, setup_walk);

                if (!ActionApplicator::is_applicable(level, try_state, agent, interaction.action)) {
                    continue;
                }

                State next_state = ActionApplicator::apply(level, try_state, agent, interaction.action);

                const ActionEffect effect =
                    ActionSemantics::compute_effect(try_state.agent_positions[agent], interaction.action);

                if (!effect.moves_box || effect.box_from != node.box || effect.box_to != nxt_box) {
                    continue;
                }

                const Position nxt_agent = next_state.agent_positions[agent];
                const int nxt_key = key_for(nxt_box, nxt_agent);

                if (parents.contains(nxt_key)) {
                    continue;
                }

                parents[nxt_key] = BoxSearchParent{current_key, true, nxt_box};
                q.push(nxt_key);
            }
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
    simulated.agent_positions[agent] = from;
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
