#include "hospital/HospitalTaskDecomposer.hpp"

#include <algorithm>
#include <functional>
#include <iostream>
#include <optional>
#include <queue>
#include <unordered_set>

namespace {
constexpr int LARGE_PENALTY = 1000;
constexpr int TRANSIT_GOAL_PENALTY = 400;
constexpr int BLOCKED_BY_CORRIDOR_PENALTY = 300;

void log_relocation_task(
    const char* phase,
    const char box_symbol,
    const int agent,
    const Position& from,
    const Position& to,
    const int candidates) {
    std::cerr << "[relocation-detected] phase=" << phase
              << " box=" << box_symbol
              << " blocker=(" << from.row << "," << from.col << ")"
              << " -> relocate_to=(" << to.row << "," << to.col << ")"
              << " agent=" << agent
              << " candidates=" << candidates
              << '\n';
}

bool can_reach_with_boxes(
    const State& state,
    const MapAnalysis& analysis,
    const Position& start,
    const Position& goal,
    const std::optional<Position>& ignored_box = std::nullopt) {
    if (start == goal) {
        return true;
    }

    std::queue<Position> q;
    std::vector<bool> visited(static_cast<std::size_t>(state.rows * state.cols), false);

    auto push_if_open = [&](const Position& p) {
        if (!analysis.is_walkable(p.row, p.col)) {
            return;
        }

        if (visited[state.index(p.row, p.col)]) {
            return;
        }

        const bool has_box = state.has_box(p.row, p.col);
        const bool is_ignored_box = ignored_box.has_value() && p == *ignored_box;
        if (has_box && p != start && p != goal && !is_ignored_box) {
            return;
        }

        visited[state.index(p.row, p.col)] = true;
        q.push(p);
    };

    push_if_open(start);

    while (!q.empty()) {
        const Position current = q.front();
        q.pop();

        if (current == goal) {
            return true;
        }

        for (const Position& neighbor : analysis.neighbors(current)) {
            push_if_open(neighbor);
        }
    }

    return false;
}

bool has_unsolved_box_goal(const Level& level, const State& state) {
    for (int row = 0; row < level.rows; ++row) {
        for (int col = 0; col < level.cols; ++col) {
            const char goal = level.goal_at(row, col);
            if (goal < 'A' || goal > 'Z') {
                continue;
            }

            if (state.box_at(row, col) != goal) {
                return true;
            }
        }
    }
    return false;
}

std::unordered_set<int> detect_transit_corridor_blockers(
    const Level& level,
    const State& state,
    const MapAnalysis& analysis,
    const std::vector<BoxRecord>& boxes) {
    std::unordered_set<int> blockers;

    for (const BoxRecord& candidate_blocker : boxes) {
        if (!analysis.is_transit_cell(candidate_blocker.pos.row, candidate_blocker.pos.col)) {
            continue;
        }

        bool blocks_other_task = false;
        for (int row = 0; row < level.rows && !blocks_other_task; ++row) {
            for (int col = 0; col < level.cols && !blocks_other_task; ++col) {
                const char goal = level.goal_at(row, col);
                if (goal < 'A' || goal > 'Z') {
                    continue;
                }

                if (state.box_at(row, col) == goal) {
                    continue;
                }

                const Position goal_pos{row, col};
                for (const BoxRecord& target : boxes) {
                    if (target.symbol != goal || target.pos == candidate_blocker.pos) {
                        continue;
                    }

                    const bool reachable_with_blocker =
                        can_reach_with_boxes(state, analysis, target.pos, goal_pos);
                    const bool reachable_without_blocker =
                        can_reach_with_boxes(state, analysis, target.pos, goal_pos, candidate_blocker.pos);

                    if (!reachable_with_blocker && reachable_without_blocker) {
                        blocks_other_task = true;
                        break;
                    }
                }
            }
        }

        if (blocks_other_task) {
            blockers.insert(state.index(candidate_blocker.pos.row, candidate_blocker.pos.col));
        }
    }

    return blockers;
}

std::optional<Position> find_non_blocking_relocation_target(
    const Level& level,
    const State& state,
    const MapAnalysis& analysis,
    const Position& from,
    const std::vector<HospitalTask>& dependency_tasks) {
    const char box_symbol = state.box_at(from.row, from.col);
    if (box_symbol == '\0') {
        return std::nullopt;
    }

    auto build_state_with_box_at = [&](const Position& pos) {
        State simulated = state;
        simulated.set_box(from.row, from.col, '\0');
        simulated.set_box(pos.row, pos.col, box_symbol);
        return simulated;
    };

    auto first_box_step_towards_goal = [&](const State& world, const Position& start, const Position& goal) -> std::optional<Position> {
        if (start == goal) {
            return goal;
        }

        std::queue<Position> q;
        std::vector<int> parent(static_cast<std::size_t>(world.rows * world.cols), -1);
        q.push(start);
        parent[world.index(start.row, start.col)] = world.index(start.row, start.col);

        while (!q.empty()) {
            const Position cur = q.front();
            q.pop();
            if (cur == goal) {
                break;
            }

            for (const Position& nxt : analysis.neighbors(cur)) {
                const int idx = world.index(nxt.row, nxt.col);
                if (parent[idx] != -1) {
                    continue;
                }
                if (world.has_box(nxt.row, nxt.col) && nxt != goal) {
                    continue;
                }
                parent[idx] = world.index(cur.row, cur.col);
                q.push(nxt);
            }
        }

        const int goal_idx = world.index(goal.row, goal.col);
        if (parent[goal_idx] == -1) {
            return std::nullopt;
        }

        Position cur = goal;
        while (parent[world.index(cur.row, cur.col)] != world.index(start.row, start.col)) {
            const int p = parent[world.index(cur.row, cur.col)];
            cur = Position{p / world.cols, p % world.cols};
        }
        return cur;
    };

    auto blocked_task_count = [&](const State& world) {
        int blocked = 0;
        for (const HospitalTask& task : dependency_tasks) {
            if (task.type != HospitalTaskType::TransportBox) {
                continue;
            }
            if (task.agent_id < 0 || task.agent_id >= world.num_agents()) {
                continue;
            }

            std::optional<Position> current_box;
            for (int row = 0; row < world.rows && !current_box.has_value(); ++row) {
                for (int col = 0; col < world.cols; ++col) {
                    if (world.box_at(row, col) == task.box_symbol) {
                        current_box = Position{row, col};
                        break;
                    }
                }
            }
            if (!current_box.has_value()) {
                continue;
            }

            const std::optional<Position> next_step =
                first_box_step_towards_goal(world, *current_box, task.destination);
            if (!next_step.has_value()) {
                ++blocked;
                continue;
            }
            if (*next_step == *current_box) {
                continue;
            }

            const int dr = next_step->row - current_box->row;
            const int dc = next_step->col - current_box->col;
            const Position setup{current_box->row - dr, current_box->col - dc};
            if (!analysis.is_walkable(setup.row, setup.col) || world.has_box(setup.row, setup.col)) {
                ++blocked;
                continue;
            }

            if (!can_reach_with_boxes(world, analysis, world.agent_positions[task.agent_id], setup, *current_box)) {
                ++blocked;
            }
        }

        return blocked;
    };

    const int baseline_blocked = blocked_task_count(state);

    auto is_blocking_at = [&](const Position& pos) {
        const State simulated = build_state_with_box_at(pos);
        return blocked_task_count(simulated) >= baseline_blocked;
    };

    std::queue<Position> frontier;
    std::unordered_set<int> visited;

    frontier.push(from);
    visited.insert(state.index(from.row, from.col));

    while (!frontier.empty()) {
        const Position current = frontier.front();
        frontier.pop();

        const State current_state = build_state_with_box_at(current);
        const std::vector<Position> candidates =
            analysis.find_relocation_candidates(current_state, current);
        for (const Position& candidate : candidates) {
            const int candidate_flat = state.index(candidate.row, candidate.col);
            if (visited.contains(candidate_flat)) {
                continue;
            }

            visited.insert(candidate_flat);
            if (!is_blocking_at(candidate)) {
                return candidate;
            }

            frontier.push(candidate);
        }
    }

    const std::vector<Position> fallback = analysis.find_relocation_candidates(state, from);
    if (!fallback.empty()) {
        return fallback.front();
    }

    return std::nullopt;
}

bool has_reachable_own_goal_access(
    const Level& level,
    const State& state,
    const MapAnalysis& analysis,
    const BoxRecord& box) {
    for (int row = 0; row < level.rows; ++row) {
        for (int col = 0; col < level.cols; ++col) {
            if (level.goal_at(row, col) != box.symbol) {
                continue;
            }

            if (can_reach_with_boxes(state, analysis, box.pos, Position{row, col})) {
                return true;
            }
        }
    }
    return false;
}
}

std::vector<HospitalTask> HospitalTaskDecomposer::decompose(
    const Level& level,
    const State& state,
    const MapAnalysis& analysis) const {
    std::vector<HospitalTask> tasks;
    std::unordered_set<int> used_box_cells;
    const bool unsolved_box_goals = has_unsolved_box_goal(level, state);

    const std::vector<BoxRecord> boxes = analysis.collect_boxes(state);
    const std::unordered_set<int> transit_blockers =
        detect_transit_corridor_blockers(level, state, analysis, boxes);

    std::vector<HospitalTask> dependency_tasks;
    for (int row = 0; row < level.rows; ++row) {
        for (int col = 0; col < level.cols; ++col) {
            const char goal = level.goal_at(row, col);
            if (goal < 'A' || goal > 'Z' || state.box_at(row, col) == goal) {
                continue;
            }

            int best_idx = -1;
            int best_cost = LARGE_PENALTY * LARGE_PENALTY;
            for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
                if (boxes[i].symbol != goal) {
                    continue;
                }
                const int cost = manhattan(boxes[i].pos, Position{row, col});
                if (cost < best_cost) {
                    best_cost = cost;
                    best_idx = i;
                }
            }
            if (best_idx == -1) {
                continue;
            }

            const BoxRecord& selected_box = boxes[best_idx];
            const int selected_agent = pick_agent_for_box(level, state, selected_box.symbol, selected_box.pos);
            if (selected_agent < 0) {
                continue;
            }

            dependency_tasks.push_back(HospitalTask{
                HospitalTaskType::TransportBox,
                selected_agent,
                selected_box.symbol,
                selected_box.pos,
                Position{row, col},
                0,
                "Dependency task for relocation blocking checks"
            });
        }
    }

    for (const BoxRecord& box : boxes) {
        const int box_flat = state.index(box.pos.row, box.pos.col);
        if (!transit_blockers.contains(box_flat)) {
            continue;
        }

        const int agent = pick_agent_for_box(level, state, box.symbol, box.pos);
        if (agent < 0) {
            continue;
        }

        const std::optional<Position> relocation =
            find_non_blocking_relocation_target(level, state, analysis, box.pos, dependency_tasks);
        if (!relocation.has_value()) {
            continue;
        }

        tasks.push_back(HospitalTask{
            HospitalTaskType::RelocateBox,
            agent,
            box.symbol,
            box.pos,
            *relocation,
            0,
            "Clear transit-corridor blocker for dependent tasks"
        });
        log_relocation_task(
            "transit-blocker",
            box.symbol,
            agent,
            box.pos,
            *relocation,
            1);
    }

    for (int row = 0; row < level.rows; ++row) {
        for (int col = 0; col < level.cols; ++col) {
            const char goal = level.goal_at(row, col);
            if (goal < 'A' || goal > 'Z') {
                continue;
            }

            Position goal_pos{row, col};
            int best_idx = -1;
            int best_cost = LARGE_PENALTY * LARGE_PENALTY;
            const bool goal_is_transit = analysis.is_transit_cell(goal_pos.row, goal_pos.col);

            for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
                const BoxRecord& box = boxes[i];
                if (box.symbol != goal) {
                    continue;
                }

                const int flat = state.index(box.pos.row, box.pos.col);
                if (used_box_cells.contains(flat)) {
                    continue;
                }

                int cost = manhattan(box.pos, goal_pos);
                if (box.in_chokepoint && !box.on_goal) {
                    cost += LARGE_PENALTY;
                }
                if (goal_is_transit && !box.on_goal) {
                    cost += TRANSIT_GOAL_PENALTY;
                }

                if (cost < best_cost) {
                    best_cost = cost;
                    best_idx = i;
                }
            }

            if (best_idx == -1) {
                continue;
            }

            const BoxRecord& selected = boxes[best_idx];
            used_box_cells.insert(state.index(selected.pos.row, selected.pos.col));

            const int agent = pick_agent_for_box(level, state, selected.symbol, selected.pos);
            if (agent < 0) {
                continue;
            }

            if (selected.in_chokepoint && !selected.on_goal
                && !has_reachable_own_goal_access(level, state, analysis, selected)) {
                const std::optional<Position> relocation =
                    find_non_blocking_relocation_target(level, state, analysis, selected.pos, dependency_tasks);
                if (relocation.has_value()) {
                    tasks.push_back(HospitalTask{
                        HospitalTaskType::RelocateBox,
                        agent,
                        selected.symbol,
                        selected.pos,
                        *relocation,
                        10,
                        "Relocate corridor-blocking box before main transport"
                    });
                    log_relocation_task(
                        "chokepoint",
                        selected.symbol,
                        agent,
                        selected.pos,
                        *relocation,
                        1);
                }
            }
            if (selected.on_goal && goal_is_transit && unsolved_box_goals) {
                const std::optional<Position> relocation =
                    find_non_blocking_relocation_target(level, state, analysis, selected.pos, dependency_tasks);
                if (relocation.has_value()) {
                    tasks.push_back(HospitalTask{
                        HospitalTaskType::RelocateBox,
                        agent,
                        selected.symbol,
                        selected.pos,
                        *relocation,
                        5,
                        "Temporarily clear transit corridor goal for other box tasks"
                    });
                    log_relocation_task(
                        "transit-goal",
                        selected.symbol,
                        agent,
                        selected.pos,
                        *relocation,
                        1);
                }
            }

            tasks.push_back(HospitalTask{
                HospitalTaskType::AssignBox,
                agent,
                selected.symbol,
                selected.pos,
                goal_pos,
                20 + best_cost,
                "Assign color-compatible transporter to box-goal pair"
            });

            tasks.push_back(HospitalTask{
                HospitalTaskType::TransportBox,
                agent,
                selected.symbol,
                selected.pos,
                goal_pos,
                100 + best_cost
                    + (transit_blockers.contains(state.index(selected.pos.row, selected.pos.col))
                           ? 0
                           : static_cast<int>(transit_blockers.size()) * BLOCKED_BY_CORRIDOR_PENALTY),
                !transit_blockers.empty() && !transit_blockers.contains(state.index(selected.pos.row, selected.pos.col))
                    ? "Transport box after corridor-clearing dependencies are resolved"
                    : (goal_is_transit
                           ? "Transport box to transit goal after non-blocking tasks"
                           : "Transport box to target hospital task location")
            });
        }
    }

    std::sort(tasks.begin(), tasks.end(), [](const HospitalTask& a, const HospitalTask& b) {
        if (a.priority != b.priority) {
            return a.priority < b.priority;
        }
        return a.agent_id < b.agent_id;
    });

    return tasks;
}

int HospitalTaskDecomposer::manhattan(const Position& a, const Position& b) noexcept {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

int HospitalTaskDecomposer::pick_agent_for_box(
    const Level& level,
    const State& state,
    const char box,
    const Position& box_pos) {
    const Color box_color = level.box_colors[box - 'A'];
    int best_agent = -1;
    int best_distance = LARGE_PENALTY * LARGE_PENALTY;

    for (int agent = 0; agent < state.num_agents(); ++agent) {
        if (level.agent_colors[agent] != box_color) {
            continue;
        }

        const int dist = manhattan(state.agent_positions[agent], box_pos);
        if (dist < best_distance) {
            best_distance = dist;
            best_agent = agent;
        }
    }

    return best_agent;
}
