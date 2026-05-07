#include "hospital/BlockerResolver.hpp"
#include <algorithm>
#include <array>
#include <limits>
#include <queue>
#include <set>
#include <vector>

namespace {
constexpr int DR[4] = {-1, 1, 0, 0};
constexpr int DC[4] = {0, 0, 1, -1};

int manhattan(const Position& a, const Position& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

bool simulated_box_at(const State& state, Position p, Position moved_from, Position parked_at) {
    if (p == moved_from) return false;
    if (p == parked_at) return true;
    return state.has_box(p.row, p.col);
}

char simulated_box_char_at(const State& state, Position p, char moved_box, Position moved_from, Position parked_at) {
    if (p == moved_from) return '\0';
    if (p == parked_at) return moved_box;
    return state.box_at(p.row, p.col);
}

bool path_to_box_neighbor_simulated(const Level& level,
                                    const State& state,
                                    Position start,
                                    Position box,
                                    Position moved_from,
                                    Position parked_at,
                                    bool boxes_block,
                                    std::vector<Position>* out_path) {
    std::vector<bool> seen(static_cast<std::size_t>(level.rows * level.cols), false);
    std::vector<Position> parent(static_cast<std::size_t>(level.rows * level.cols), Position{-1, -1});
    std::queue<Position> q;
    q.push(start);
    seen[static_cast<std::size_t>(level.index(start.row, start.col))] = true;
    while (!q.empty()) {
        Position cur = q.front();
        q.pop();
        if (manhattan(cur, box) == 1) {
            if (out_path) {
                out_path->clear();
                for (Position p = cur; !(p == Position{-1, -1}); p = parent[static_cast<std::size_t>(level.index(p.row, p.col))]) {
                    out_path->push_back(p);
                    if (p == start) break;
                }
                std::reverse(out_path->begin(), out_path->end());
            }
            return true;
        }
        for (int i = 0; i < 4; ++i) {
            Position nxt{cur.row + DR[i], cur.col + DC[i]};
            if (!level.in_bounds(nxt.row, nxt.col) || level.is_wall(nxt.row, nxt.col)) continue;
            if (boxes_block && simulated_box_at(state, nxt, moved_from, parked_at) && !(nxt == box)) continue;
            const int idx = level.index(nxt.row, nxt.col);
            if (seen[static_cast<std::size_t>(idx)]) continue;
            seen[static_cast<std::size_t>(idx)] = true;
            parent[static_cast<std::size_t>(idx)] = cur;
            q.push(nxt);
        }
    }
    return false;
}

bool path_to_box_neighbor(const Level& level, const State& state, Position start, Position box, bool boxes_block, std::vector<Position>* out_path) {
    return path_to_box_neighbor_simulated(level, state, start, box, Position{-1, -1}, Position{-1, -1}, boxes_block, out_path);
}

bool reachable_without_boxes(const Level& level,
                             const State& state,
                             Position start,
                             Position goal,
                             char moved_box,
                             Position moved_from,
                             Position parked_at) {
    std::vector<bool> seen(static_cast<std::size_t>(level.rows * level.cols), false);
    std::queue<Position> q;
    q.push(start);
    seen[static_cast<std::size_t>(level.index(start.row, start.col))] = true;
    while (!q.empty()) {
        const Position cur = q.front();
        q.pop();
        if (cur == goal) return true;
        for (int i = 0; i < 4; ++i) {
            const Position nxt{cur.row + DR[i], cur.col + DC[i]};
            if (!level.in_bounds(nxt.row, nxt.col) || level.is_wall(nxt.row, nxt.col)) continue;
            const char box = simulated_box_char_at(state, nxt, moved_box, moved_from, parked_at);
            if (box != '\0' && !(nxt == start) && !(nxt == goal)) continue;
            const int idx = level.index(nxt.row, nxt.col);
            if (seen[static_cast<std::size_t>(idx)]) continue;
            seen[static_cast<std::size_t>(idx)] = true;
            q.push(nxt);
        }
    }
    return false;
}

Position find_box_after_parking(const State& state, char box, char moved_box, Position moved_from, Position parked_at) {
    if (box == moved_box) return parked_at;
    for (int r = 0; r < state.rows; ++r) {
        for (int c = 0; c < state.cols; ++c) {
            const Position p{r, c};
            if (p == moved_from) continue;
            if (state.box_at(r, c) == box) return p;
        }
    }
    return Position{-1, -1};
}

bool parking_preserves_future_reachability(const Level& level,
                                           const State& state,
                                           const LevelAnalysis& analysis,
                                           char moved_box,
                                           Position moved_from,
                                           Position parked_at,
                                           Position forbidden) {
    if (!level.in_bounds(parked_at.row, parked_at.col) || level.is_wall(parked_at.row, parked_at.col)) return false;
    if (parked_at == moved_from || parked_at == forbidden) return false;
    if (state.has_box(parked_at.row, parked_at.col) && !(parked_at == moved_from)) return false;
    if (level.goal_at(parked_at.row, parked_at.col) != '\0') return false;
    if (analysis.rows == level.rows && analysis.cols == level.cols && !analysis.cells.empty()) {
        const CellInfo& cell = analysis.at(parked_at);
        if (cell.is_chokepoint || cell.is_articulation) return false;
    }

    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const char goal = level.goal_at(r, c);
            if (goal < 'A' || goal > 'Z') continue;
            if (simulated_box_char_at(state, Position{r, c}, moved_box, moved_from, parked_at) == goal) continue;

            const Position box_pos = find_box_after_parking(state, goal, moved_box, moved_from, parked_at);
            if (box_pos.row == -1) continue;
            const Position goal_pos{r, c};
            if (!reachable_without_boxes(level, state, box_pos, goal_pos, moved_box, moved_from, parked_at)) return false;

            const Color goal_color = level.box_colors[static_cast<std::size_t>(goal - 'A')];
            bool any_agent_can_reach = false;
            for (int a = 0; a < state.num_agents(); ++a) {
                if (level.agent_colors[static_cast<std::size_t>(a)] != goal_color) continue;
                if (path_to_box_neighbor_simulated(level, state, state.agent_positions[static_cast<std::size_t>(a)], box_pos,
                                                   moved_from, parked_at, true, nullptr)) {
                    any_agent_can_reach = true;
                    break;
                }
            }
            if (!any_agent_can_reach) return false;
        }
    }
    return true;
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

bool on_future_coarse_route(const Level& level, const State& state, Position p) {
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
            const auto route = coarse_route(active_box, Position{r, c});
            if (std::find(route.begin(), route.end(), p) != route.end()) return true;
        }
    }
    return false;
}

int base_parking_score(const Level& level, const State& state, const LevelAnalysis& analysis, Position box_pos, Position p) {
    int score = analysis.at(p).parking_score - manhattan(box_pos, p);
    score += (level.goal_at(p.row, p.col) == '\0') ? 20 : -1000;
    if (analysis.rows == state.rows && analysis.cols == state.cols && !analysis.cells.empty()) {
        const auto& cell = analysis.at(p);
        if (cell.is_chokepoint) score -= 30000;
        if (cell.is_articulation) score -= 30000;
        if (cell.is_corridor) score -= 1000;
    }
    if (on_future_coarse_route(level, state, p)) score -= 500;
    return score;
}

Position best_parking_for(const Level& level, const State& state, const LevelAnalysis& analysis, char moved_box, Position box_pos, Position forbidden, int* out_score = nullptr) {
    Position best = analysis.parking_cells.front();
    int best_score = std::numeric_limits<int>::min();
    Position fallback = best;
    int fallback_score = std::numeric_limits<int>::min();
    for (const Position& p : analysis.parking_cells) {
        const int score = base_parking_score(level, state, analysis, box_pos, p);
        if (score > fallback_score && p != box_pos && p != forbidden) { fallback_score = score; fallback = p; }
        if (!parking_preserves_future_reachability(level, state, analysis, moved_box, box_pos, p, forbidden)) continue;
        if (score > best_score) { best_score = score; best = p; }
    }
    if (best_score == std::numeric_limits<int>::min()) {
        best = fallback;
        best_score = fallback_score;
    }
    if (out_score) *out_score = best_score;
    return best;
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
            Position best_park = best_parking_for(level, state, analysis, b, t.box_pos, Position{-1, -1}, &best_score);
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

    // If a compatible agent cannot reach a box because another box gates the
    // approach corridor, create a parking task for the first blocking box on an
    // otherwise passable route. This catches simple cases where a box is not on
    // the delivery route but still prevents the agent from getting to the box.
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

            int agent = -1;
            int best_dist = std::numeric_limits<int>::max();
            const Color goal_color = level.box_colors[static_cast<std::size_t>(goal - 'A')];
            for (int a = 0; a < state.num_agents(); ++a) {
                if (level.agent_colors[static_cast<std::size_t>(a)] != goal_color) continue;
                const int d = manhattan(state.agent_positions[static_cast<std::size_t>(a)], active_box);
                if (d < best_dist) { best_dist = d; agent = a; }
            }
            if (agent < 0) continue;
            const Position agent_pos = state.agent_positions[static_cast<std::size_t>(agent)];
            if (path_to_box_neighbor(level, state, agent_pos, active_box, true, nullptr)) continue;

            std::vector<Position> passable_path;
            if (!path_to_box_neighbor(level, state, agent_pos, active_box, false, &passable_path)) continue;
            for (const Position& p : passable_path) {
                const char blocker = state.box_at(p.row, p.col);
                if (blocker == '\0' || blocker == goal || already_selected.count(blocker)) continue;
                if (level.goal_at(p.row, p.col) != '\0') continue;

                int blocker_agent = -1;
                int blocker_best_dist = std::numeric_limits<int>::max();
                const Color blocker_color = level.box_colors[static_cast<std::size_t>(blocker - 'A')];
                for (int a = 0; a < state.num_agents(); ++a) {
                    if (level.agent_colors[static_cast<std::size_t>(a)] != blocker_color) continue;
                    const int d = manhattan(state.agent_positions[static_cast<std::size_t>(a)], p);
                    if (d < blocker_best_dist) { blocker_best_dist = d; blocker_agent = a; }
                }
                if (blocker_agent < 0) break;

                Task t;
                t.type = TaskType::MoveBlockingBoxToParking;
                t.task_id = next_task_id++;
                t.box_id = blocker;
                t.box_pos = p;
                int selected_score = 0;
                t.parking_pos = best_parking_for(level, state, analysis, blocker, p, Position{r, c}, &selected_score);
                t.goal_pos = t.parking_pos;
                t.agent_id = blocker_agent;
                t.priority = selected_score + 100;
                t.unblocks_box_id = goal;
                t.debug_label = "agent_access_blocker_for_" + std::string(1, goal);
                tasks.push_back(t);
                already_selected.insert(blocker);
                break;
            }
        }
    }

    // Additional blocker extraction: for each unsatisfied delivery route,
    // identify only boxes directly on the coarse route and park them.
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
                const Position q = rp;
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
                t.unblocks_box_id = goal;
                t.debug_label = "route_blocker_for_" + std::string(1, goal);

                int best_score = 0;
                const Position best_park = best_parking_for(level, state, analysis, b, q, Position{r, c}, &best_score);
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
    return tasks;
}
