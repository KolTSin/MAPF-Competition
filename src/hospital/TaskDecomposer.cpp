#include "hospital/TaskDecomposer.hpp"

#include <algorithm>
#include <iostream>
#include <optional>
#include <queue>
#include <unordered_set>

namespace {
constexpr int LARGE_PENALTY = 1000;

int local_manhattan(const Position& a, const Position& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
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

        const int idx = state.index(p.row, p.col);
        if (visited[idx]) {
            return;
        }

        const bool is_ignored = ignored_box.has_value() && p == *ignored_box;
        if (state.has_box(p.row, p.col) && p != start && p != goal && !is_ignored) {
            return;
        }

        visited[idx] = true;
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

bool task_has_access(
    const Level& level,
    const State& state,
    const MapAnalysis& analysis,
    const Task& task) {
    (void) level;

    if (task.type != TaskType::TransportBox) {
        return true;
    }

    if (task.agent_id < 0 || task.agent_id >= state.num_agents()) {
        return false;
    }

    std::optional<Position> box_pos = find_current_box(state, task.box_symbol, task.source);
    if (!box_pos.has_value()) {
        return false;
    }

    const Position box = *box_pos;
    const Position goal = task.destination;
    const Position agent_pos = state.agent_positions[task.agent_id];

    for (const Position& next_box_pos : analysis.neighbors(box)) {
        if (!analysis.is_walkable(next_box_pos.row, next_box_pos.col)) {
            continue;
        }

        if (state.has_box(next_box_pos.row, next_box_pos.col) && next_box_pos != goal) {
            continue;
        }

        if (local_manhattan(next_box_pos, goal) > local_manhattan(box, goal)) {
            continue;
        }

        const int dr = next_box_pos.row - box.row;
        const int dc = next_box_pos.col - box.col;
        const Position setup{box.row - dr, box.col - dc};

        if (!analysis.is_walkable(setup.row, setup.col)) {
            continue;
        }

        if (state.has_box(setup.row, setup.col) && setup != box) {
            continue;
        }

        if (can_reach_with_boxes(state, analysis, agent_pos, setup, box)) {
            return true;
        }
    }

    return false;
}

std::optional<Position> find_goal_for_symbol(const Level& level, const char symbol) {
    for (int row = 0; row < level.rows; ++row) {
        for (int col = 0; col < level.cols; ++col) {
            if (level.goal_at(row, col) == symbol) {
                return Position{row, col};
            }
        }
    }
    return std::nullopt;
}

std::optional<Position> find_safe_parking_cell(
    const Level& level,
    const State& state,
    const MapAnalysis& analysis,
    const BoxRecord& blocker,
    const std::vector<Task>& goal_tasks,
    const std::vector<int>& blocked_task_indices) {
    const std::vector<Position> candidates = analysis.find_relocation_candidates(state, blocker.pos);
    if (candidates.empty()) {
        return std::nullopt;
    }

    const std::optional<Position> blocker_goal = find_goal_for_symbol(level, blocker.symbol);

    auto simulate_blocker_at = [&](const Position& candidate) {
        State simulated = state;
        simulated.set_box(blocker.pos.row, blocker.pos.col, '\0');
        simulated.set_box(candidate.row, candidate.col, blocker.symbol);
        return simulated;
    };

    int best_cost = LARGE_PENALTY * LARGE_PENALTY;
    std::optional<Position> best;

    for (const Position& candidate : candidates) {
        if (!analysis.is_walkable(candidate.row, candidate.col)) {
            continue;
        }

        if (candidate != blocker.pos && state.has_box(candidate.row, candidate.col)) {
            continue;
        }

        State simulated = simulate_blocker_at(candidate);

        bool unblocks_required_tasks = true;
        for (const int blocked_idx : blocked_task_indices) {
            if (!task_has_access(level, simulated, analysis, goal_tasks[blocked_idx])) {
                unblocks_required_tasks = false;
                break;
            }
        }
        if (!unblocks_required_tasks) {
            continue;
        }

        if (blocker_goal.has_value() && !can_reach_with_boxes(simulated, analysis, candidate, *blocker_goal)) {
            continue;
        }

        int cost = local_manhattan(blocker.pos, candidate);
        if (analysis.is_transit_cell(candidate.row, candidate.col)) {
            cost += 5000;
        }
        if (analysis.is_chokepoint(candidate.row, candidate.col)) {
            cost += 5000;
        }
        if (level.goal_at(candidate.row, candidate.col) != '\0') {
            cost += 10000;
        }

        if (!best.has_value() || cost < best_cost) {
            best = candidate;
            best_cost = cost;
        }
    }

    return best;
}
}

std::vector<Task> TaskDecomposer::decompose(
    const Level& level,
    const State& state,
    const MapAnalysis& analysis) const {
    std::vector<Task> tasks;
    const std::vector<BoxRecord> boxes = analysis.collect_boxes(state);

    std::vector<Task> goal_tasks;
    for (int row = 0; row < level.rows; ++row) {
        for (int col = 0; col < level.cols; ++col) {
            const char goal = level.goal_at(row, col);
            if (goal < 'A' || goal > 'Z') {
                continue;
            }
            if (state.box_at(row, col) == goal) {
                continue;
            }

            int best_idx = -1;
            int best_cost = LARGE_PENALTY * LARGE_PENALTY;
            for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
                if (boxes[i].symbol != goal) {
                    continue;
                }

                const int dist = manhattan(boxes[i].pos, Position{row, col});
                if (dist < best_cost) {
                    best_cost = dist;
                    best_idx = i;
                }
            }
            if (best_idx < 0) {
                continue;
            }

            const BoxRecord& selected = boxes[best_idx];
            const int agent = pick_agent_for_box(level, state, selected.symbol, selected.pos);
            if (agent < 0) {
                continue;
            }

            goal_tasks.push_back(Task{
                TaskType::TransportBox,
                agent,
                selected.symbol,
                selected.pos,
                Position{row, col},
                best_cost,
                "Transport box to goal",
                TaskPhase::SolvePrimaryGoals,
                Position{-1, -1}
            });
        }
    }

    std::vector<bool> is_blocker(boxes.size(), false);
    std::vector<std::vector<int>> blocker_unblocked_tasks(boxes.size());

    for (std::size_t i = 0; i < boxes.size(); ++i) {
        const BoxRecord& candidate = boxes[i];

        State without = state;
        without.set_box(candidate.pos.row, candidate.pos.col, '\0');

        for (int t = 0; t < static_cast<int>(goal_tasks.size()); ++t) {
            const Task& task = goal_tasks[t];
            if (task.box_symbol == candidate.symbol && task.source == candidate.pos) {
                continue;
            }

            const bool with_box = task_has_access(level, state, analysis, task);
            const bool without_box = task_has_access(level, without, analysis, task);

            if (!with_box && without_box) {
                is_blocker[i] = true;
                blocker_unblocked_tasks[i].push_back(t);
            }
        }
    }

    for (int t = 0; t < static_cast<int>(goal_tasks.size()); ++t) {
        const Task& task = goal_tasks[t];
        if (task_has_access(level, state, analysis, task)) {
            continue;
        }

        for (std::size_t i = 0; i < boxes.size(); ++i) {
            if (boxes[i].symbol == task.box_symbol && boxes[i].pos == task.source) {
                is_blocker[i] = true;
                blocker_unblocked_tasks[i].push_back(t);
                break;
            }
        }
    }

    std::unordered_set<int> relocated_box_cells;
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        if (!is_blocker[i]) {
            continue;
        }

        const BoxRecord& blocker = boxes[i];
        const int agent = pick_agent_for_box(level, state, blocker.symbol, blocker.pos);
        if (agent < 0) {
            continue;
        }

        const std::optional<Position> parking = find_safe_parking_cell(
            level,
            state,
            analysis,
            blocker,
            goal_tasks,
            blocker_unblocked_tasks[i]);
        if (!parking.has_value()) {
            continue;
        }

        const std::optional<Position> final_goal = find_goal_for_symbol(level, blocker.symbol);
        if (!final_goal.has_value()) {
            continue;
        }

        tasks.push_back(Task{
            TaskType::RelocateBox,
            agent,
            blocker.symbol,
            blocker.pos,
            *parking,
            0,
            "Temporarily clear blocker",
            TaskPhase::ClearBlockers,
            *final_goal
        });

        tasks.push_back(Task{
            TaskType::TransportBox,
            agent,
            blocker.symbol,
            *parking,
            *final_goal,
            1000,
            "Restore relocated box to final goal",
            TaskPhase::RestoreRelocatedBoxes,
            *final_goal
        });

        relocated_box_cells.insert(state.index(blocker.pos.row, blocker.pos.col));
    }

    for (const Task& goal_task : goal_tasks) {
        std::optional<Position> current_box = find_current_box(state, goal_task.box_symbol, goal_task.source);
        if (!current_box.has_value()) {
            continue;
        }

        if (relocated_box_cells.contains(state.index(current_box->row, current_box->col))) {
            continue;
        }

        Task task = goal_task;
        task.source = *current_box;
        tasks.push_back(task);
    }

    std::sort(tasks.begin(), tasks.end(), [](const Task& a, const Task& b) {
        if (a.phase != b.phase) {
            return static_cast<int>(a.phase) < static_cast<int>(b.phase);
        }
        if (a.priority != b.priority) {
            return a.priority < b.priority;
        }
        return a.agent_id < b.agent_id;
    });

    for (const Task& task : tasks) {
        std::cerr << "[task] phase=" << static_cast<int>(task.phase)
                  << " type=" << static_cast<int>(task.type)
                  << " agent=" << task.agent_id
                  << " box=" << task.box_symbol
                  << " from=(" << task.source.row << "," << task.source.col << ")"
                  << " to=(" << task.destination.row << "," << task.destination.col << ")"
                  << " reason=" << task.reason
                  << '\n';
    }

    return tasks;
}

int TaskDecomposer::manhattan(const Position& a, const Position& b) noexcept {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

int TaskDecomposer::pick_agent_for_box(
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
