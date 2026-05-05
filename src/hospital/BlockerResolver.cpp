#include "hospital/BlockerResolver.hpp"
#include <array>
#include <limits>
#include <set>
#include <vector>

namespace {
int manhattan(const Position& a, const Position& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

std::vector<Position> coarse_route(Position from, Position to) {
    std::vector<Position> route;
    Position cur = from;
    route.push_back(cur);
    while (cur.row != to.row) {
        cur.row += (to.row > cur.row) ? 1 : -1;
        route.push_back(cur);
    }
    while (cur.col != to.col) {
        cur.col += (to.col > cur.col) ? 1 : -1;
        route.push_back(cur);
    }
    return route;
}
}

std::vector<Task> BlockerResolver::generate_blocker_tasks(const Level& level,
                                                          const State& state,
                                                          const LevelAnalysis& analysis,
                                                          int& next_task_id) const {
    std::vector<Task> tasks;
    if (analysis.parking_cells.empty()) return tasks;
    std::set<char> already_selected;
    std::array<bool, 26> needed_for_goal{};
    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const char g = level.goal_at(r, c);
            if (g < 'A' || g > 'Z') continue;
            if (state.box_at(r, c) != g) {
                needed_for_goal[static_cast<std::size_t>(g - 'A')] = true;
            }
        }
    }

    for (int r = 0; r < state.rows; ++r) {
        for (int c = 0; c < state.cols; ++c) {
            const char b = state.box_at(r, c);
            if (b == '\0') continue;
            if (level.goal_at(r, c) != '\0') continue;
            if (needed_for_goal[static_cast<std::size_t>(b - 'A')]) continue;

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
            already_selected.insert(b);
        }
    }

    // Additional blocker extraction: for each unsatisfied delivery route,
    // identify other boxes on/near the coarse route and park them.
    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const char goal = level.goal_at(r, c);
            if (goal < 'A' || goal > 'Z') continue;
            if (state.box_at(r, c) == goal) continue;

            Position active_box{-1, -1};
            for (int br = 0; br < state.rows; ++br) {
                for (int bc = 0; bc < state.cols; ++bc) {
                    if (state.box_at(br, bc) == goal) { active_box = Position{br, bc}; break; }
                }
                if (active_box.row != -1) break;
            }
            if (active_box.row == -1) continue;
            const std::vector<Position> route = coarse_route(active_box, Position{r, c});
            for (const Position& rp : route) {
                for (int dr = -1; dr <= 1; ++dr) {
                    for (int dc = -1; dc <= 1; ++dc) {
                        const Position q{rp.row + dr, rp.col + dc};
                        if (!state.in_bounds(q.row, q.col)) continue;
                        const char b = state.box_at(q.row, q.col);
                        if (b == '\0' || b == goal) continue;
                        if (already_selected.count(b)) continue;
                        if (level.goal_at(q.row, q.col) != '\0') continue;

                        Task t;
                        t.type = TaskType::MoveBlockingBoxToParking;
                        t.task_id = next_task_id++;
                        t.box_id = b;
                        t.box_pos = q;
                        t.debug_label = "route_blocker_for_" + std::string(1, goal);

                        int best_score = std::numeric_limits<int>::min();
                        Position best_park = analysis.parking_cells.front();
                        for (const Position& p : analysis.parking_cells) {
                            int score = analysis.at(p).parking_score - manhattan(q, p);
                            if (p == q || p == Position{r, c}) score -= 50;
                            if (score > best_score) { best_score = score; best_park = p; }
                        }
                        t.parking_pos = best_park;
                        t.goal_pos = best_park;

                        int best_agent = -1;
                        int best_dist = std::numeric_limits<int>::max();
                        const Color box_color = level.box_colors[static_cast<std::size_t>(b - 'A')];
                        for (int a = 0; a < state.num_agents(); ++a) {
                            if (level.agent_colors[static_cast<std::size_t>(a)] != box_color) continue;
                            const int dist = manhattan(state.agent_positions[static_cast<std::size_t>(a)], q);
                            if (dist < best_dist) { best_dist = dist; best_agent = a; }
                        }
                        if (best_agent < 0) continue;
                        t.agent_id = best_agent;
                        t.priority = best_score + 20;
                        tasks.push_back(t);
                        already_selected.insert(b);
                    }
                }
            }
        }
    }
    return tasks;
}
