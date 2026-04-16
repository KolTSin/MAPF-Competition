#include "hospital/HospitalTaskDecomposer.hpp"

#include <algorithm>
#include <unordered_set>

namespace {
constexpr int LARGE_PENALTY = 1000;
}

std::vector<HospitalTask> HospitalTaskDecomposer::decompose(
    const Level& level,
    const State& state,
    const MapAnalysis& analysis) const {
    std::vector<HospitalTask> tasks;
    std::unordered_set<int> used_box_cells;

    const std::vector<BoxRecord> boxes = analysis.collect_boxes(state);

    for (int row = 0; row < level.rows; ++row) {
        for (int col = 0; col < level.cols; ++col) {
            const char goal = level.goal_at(row, col);
            if (goal < 'A' || goal > 'Z') {
                continue;
            }

            Position goal_pos{row, col};
            int best_idx = -1;
            int best_cost = LARGE_PENALTY * LARGE_PENALTY;

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
                100 + best_cost,
                "Transport box to target hospital task location"
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
