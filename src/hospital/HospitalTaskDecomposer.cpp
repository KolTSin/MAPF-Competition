#include "hospital/HospitalTaskDecomposer.hpp"

#include <algorithm>
#include <optional>
#include <queue>
#include <unordered_set>

namespace {
constexpr int LARGE_PENALTY = 1000;
constexpr int TRANSIT_GOAL_PENALTY = 400;
constexpr int BLOCKED_BY_CORRIDOR_PENALTY = 300;

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

    for (const BoxRecord& box : boxes) {
        const int box_flat = state.index(box.pos.row, box.pos.col);
        if (!transit_blockers.contains(box_flat)) {
            continue;
        }

        const int agent = pick_agent_for_box(level, state, box.symbol, box.pos);
        if (agent < 0) {
            continue;
        }

        const std::vector<Position> relocation = analysis.find_relocation_candidates(state, box.pos);
        if (relocation.empty()) {
            continue;
        }

        tasks.push_back(HospitalTask{
            HospitalTaskType::RelocateBox,
            agent,
            box.symbol,
            box.pos,
            relocation.front(),
            0,
            "Clear transit-corridor blocker for dependent tasks"
        });
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

            if (selected.in_chokepoint && !selected.on_goal) {
                std::vector<Position> relocation = analysis.find_relocation_candidates(state, selected.pos);
                if (!relocation.empty()) {
                    tasks.push_back(HospitalTask{
                        HospitalTaskType::RelocateBox,
                        agent,
                        selected.symbol,
                        selected.pos,
                        relocation.front(),
                        10,
                        "Relocate corridor-blocking box before main transport"
                    });
                }
            }
            if (selected.on_goal && goal_is_transit && unsolved_box_goals) {
                std::vector<Position> relocation = analysis.find_relocation_candidates(state, selected.pos);
                if (!relocation.empty()) {
                    tasks.push_back(HospitalTask{
                        HospitalTaskType::RelocateBox,
                        agent,
                        selected.symbol,
                        selected.pos,
                        relocation.front(),
                        5,
                        "Temporarily clear transit corridor goal for other box tasks"
                    });
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
