#include "hospital/BlockerResolver.hpp"
#include <limits>

namespace {
int manhattan(const Position& a, const Position& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}
}

std::vector<Task> BlockerResolver::generate_blocker_tasks(const Level& level,
                                                          const State& state,
                                                          const LevelAnalysis& analysis,
                                                          int& next_task_id) const {
    std::vector<Task> tasks;
    if (analysis.parking_cells.empty()) return tasks;

    for (int r = 0; r < state.rows; ++r) {
        for (int c = 0; c < state.cols; ++c) {
            const char b = state.box_at(r, c);
            if (b == '\0') continue;
            if (level.goal_at(r, c) != '\0') continue;

            Task t;
            t.type = TaskType::MoveBlockingBoxToParking;
            t.task_id = next_task_id++;
            t.box_id = b;
            t.box_pos = Position{r, c};
            int best_score = std::numeric_limits<int>::min();
            Position best_park = analysis.parking_cells.front();
            for (const Position& p : analysis.parking_cells) {
                int score = 0;
                score += (level.goal_at(p.row, p.col) == '\0') ? 20 : -50;
                if (analysis.rows == state.rows && analysis.cols == state.cols && !analysis.cells.empty()) {
                    const auto& cell = analysis.at(p);
                    if (cell.is_chokepoint) score -= 30;
                    if (cell.is_corridor) score -= 10;
                    score += cell.parking_score;
                }
                score -= manhattan(Position{r,c}, p);
                if (score > best_score) {
                    best_score = score;
                    best_park = p;
                }
            }
            t.parking_pos = best_park;
            t.goal_pos = t.parking_pos;
            int best_agent = -1;
            int best_dist = std::numeric_limits<int>::max();
            const Color box_color = level.box_colors[static_cast<std::size_t>(b - 'A')];
            for (int a = 0; a < state.num_agents(); ++a) {
                if (level.agent_colors[static_cast<std::size_t>(a)] != box_color) continue;
                const int dist = manhattan(state.agent_positions[static_cast<std::size_t>(a)], t.box_pos);
                if (dist < best_dist) {
                    best_dist = dist;
                    best_agent = a;
                }
            }
            t.agent_id = (best_agent >= 0) ? best_agent : 0;
            t.priority = best_score;
            tasks.push_back(t);
        }
    }
    return tasks;
}
