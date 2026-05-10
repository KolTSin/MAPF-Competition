#include "analysis/ParkingCellAnalyzer.hpp"
#include <algorithm>
#include <iostream>
#include <limits>
#include <queue>

namespace {
// Four-neighbor offsets used to inspect the immediate surroundings of a
// candidate parking cell. Parking quality depends not only on the cell itself
// but also on whether parking there would sit beside goals or chokepoints.
constexpr int DR[4] = {-1, 1, 0, 0};
constexpr int DC[4] = {0, 0, -1, 1};

constexpr int kAgentPathCellPenalty = 5000;
constexpr int kAgentPathExtraVisitPenalty = 250;
constexpr int kHardRejectScore = -100000;

int manhattan(Position a, Position b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

int agent_path_penalty(Position p, const std::vector<AgentPlan>& initial_agent_plans) {
    int visits = 0;
    for (const AgentPlan& plan : initial_agent_plans) {
        for (const Position& pos : plan.positions) {
            if (pos == p) ++visits;
        }
    }

    if (visits == 0) return 0;
    return kAgentPathCellPenalty + (visits - 1) * kAgentPathExtraVisitPenalty;
}

Position find_box(const State& state, char box) {
    for (int r = 0; r < state.rows; ++r) {
        for (int c = 0; c < state.cols; ++c) {
            if (state.box_at(r, c) == box) return Position{r, c};
        }
    }
    return Position{-1, -1};
}

std::vector<Position> shortest_path_ignoring_boxes(const Level& level, Position start, Position goal) {
    if (!level.in_bounds(start.row, start.col) || !level.in_bounds(goal.row, goal.col)) return {};
    if (level.is_wall(start.row, start.col) || level.is_wall(goal.row, goal.col)) return {};

    std::vector<bool> seen(static_cast<std::size_t>(level.rows * level.cols), false);
    std::vector<Position> parent(static_cast<std::size_t>(level.rows * level.cols), Position{-1, -1});
    std::queue<Position> q;
    q.push(start);
    seen[static_cast<std::size_t>(level.index(start.row, start.col))] = true;

    while (!q.empty()) {
        const Position cur = q.front();
        q.pop();
        if (cur == goal) break;

        for (int i = 0; i < 4; ++i) {
            const Position nxt{cur.row + DR[i], cur.col + DC[i]};
            if (!level.in_bounds(nxt.row, nxt.col) || level.is_wall(nxt.row, nxt.col)) continue;
            const int idx = level.index(nxt.row, nxt.col);
            if (seen[static_cast<std::size_t>(idx)]) continue;
            seen[static_cast<std::size_t>(idx)] = true;
            parent[static_cast<std::size_t>(idx)] = cur;
            q.push(nxt);
        }
    }

    const int goal_idx = level.index(goal.row, goal.col);
    if (!seen[static_cast<std::size_t>(goal_idx)]) return {};

    std::vector<Position> path;
    for (Position p = goal; !(p == Position{-1, -1}); p = parent[static_cast<std::size_t>(level.index(p.row, p.col))]) {
        path.push_back(p);
        if (p == start) break;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Position> shortest_path_to_box_neighbor(const Level& level, Position start, Position box) {
    if (!level.in_bounds(start.row, start.col) || level.is_wall(start.row, start.col)) return {};

    std::vector<bool> seen(static_cast<std::size_t>(level.rows * level.cols), false);
    std::vector<Position> parent(static_cast<std::size_t>(level.rows * level.cols), Position{-1, -1});
    std::queue<Position> q;
    q.push(start);
    seen[static_cast<std::size_t>(level.index(start.row, start.col))] = true;

    Position target{-1, -1};
    while (!q.empty() && target.row == -1) {
        const Position cur = q.front();
        q.pop();
        if (manhattan(cur, box) == 1) {
            target = cur;
            break;
        }

        for (int i = 0; i < 4; ++i) {
            const Position nxt{cur.row + DR[i], cur.col + DC[i]};
            if (!level.in_bounds(nxt.row, nxt.col) || level.is_wall(nxt.row, nxt.col)) continue;
            const int idx = level.index(nxt.row, nxt.col);
            if (seen[static_cast<std::size_t>(idx)]) continue;
            seen[static_cast<std::size_t>(idx)] = true;
            parent[static_cast<std::size_t>(idx)] = cur;
            q.push(nxt);
        }
    }
    if (target.row == -1) return {};

    std::vector<Position> path;
    for (Position p = target; !(p == Position{-1, -1}); p = parent[static_cast<std::size_t>(level.index(p.row, p.col))]) {
        path.push_back(p);
        if (p == start) break;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

int count_path_visits(Position p, const std::vector<Position>& path) {
    return static_cast<int>(std::count(path.begin(), path.end(), p));
}

int future_route_visits(Position p, const Level& level, const State& state) {
    int visits = 0;
    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const char goal = level.goal_at(r, c);
            if (goal < 'A' || goal > 'Z') continue;
            if (state.box_at(r, c) == goal) continue;

            const Position box = find_box(state, goal);
            if (box.row == -1) continue;
            visits += count_path_visits(p, shortest_path_ignoring_boxes(level, box, Position{r, c}));

            const Color box_color = level.box_colors[static_cast<std::size_t>(goal - 'A')];
            for (int agent = 0; agent < state.num_agents(); ++agent) {
                if (level.agent_colors[static_cast<std::size_t>(agent)] != box_color) continue;
                visits += count_path_visits(p, shortest_path_to_box_neighbor(level, state.agent_positions[static_cast<std::size_t>(agent)], box));
            }
        }
    }
    return visits;
}

int nearest_goal_or_box_distance(Position p, const Level& level, const State& state) {
    int best = std::numeric_limits<int>::max();
    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const char goal = level.goal_at(r, c);
            if (goal != '\0' && goal != ' ') best = std::min(best, manhattan(p, Position{r, c}));
            if (state.has_box(r, c)) best = std::min(best, manhattan(p, Position{r, c}));
        }
    }
    return best == std::numeric_limits<int>::max() ? 0 : best;
}
}

int ParkingCellAnalyzer::score_parking_cell(Position p, const Level& level, const State& state, const LevelAnalysis& analysis) const {
    static const std::vector<AgentPlan> kNoInitialAgentPlans;
    return score_parking_cell(p, level, state, analysis, kNoInitialAgentPlans);
}

int ParkingCellAnalyzer::score_parking_cell(Position p,
                                            const Level& level,
                                            const State& state,
                                            const LevelAnalysis& analysis,
                                            const std::vector<AgentPlan>& initial_agent_plans) const {
    // Hard rejection: cells outside the map, walls, occupied cells, and goals
    // are invalid parking spots. Returning the same very negative score keeps
    // them below every usable candidate while preserving a numeric API.
    if (!level.in_bounds(p.row, p.col) || level.is_wall(p.row, p.col)) return kHardRejectScore;
    if (state.has_box(p.row, p.col)) return kHardRejectScore;

    const CellInfo& cell = analysis.at(p);
    if (cell.is_goal_cell) return kHardRejectScore;

    // Base score: any empty non-goal free cell may be needed as a fallback, but
    // isolated side pockets are much better parking than transit structures.
    int score = 5000;
    if (cell.is_room) score += 4000;
    if (cell.is_dead_end) score += 9000;
    if (cell.is_intersection) score -= 20000;

    // Topology penalties: corridors and connectors are often unavoidable in
    // narrow levels, so keep them rankable instead of collapsing them all into a
    // giant negative bucket. Dead ends are deliberately exempt from the connector
    // penalties because they are exactly the kind of low-traffic side pocket we
    // want to reward for temporary storage.
    if (!cell.is_dead_end) {
        if (cell.is_articulation) score -= 2500;
        if (cell.is_chokepoint) score -= 1500;
    }
    if (cell.is_corridor) score -= 1000;

    // Neighborhood scan: avoid parking next to goals and fragile connectors when
    // comparable alternatives exist, but do not punish true dead ends for being
    // adjacent to the corridor that enters them.
    bool adjacent_chokepoint = false;
    bool adjacent_goal = false;
    for (int i = 0; i < 4; ++i) {
        Position n{p.row + DR[i], p.col + DC[i]};
        if (!level.in_bounds(n.row, n.col) || level.is_wall(n.row, n.col)) continue;

        const CellInfo& neighbor = analysis.at(n);
        adjacent_chokepoint = adjacent_chokepoint || neighbor.is_chokepoint;
        adjacent_goal = adjacent_goal || neighbor.is_goal_cell;
    }

    if (adjacent_goal) score -= 2000;
    else score += 250;
    if (adjacent_chokepoint && !cell.is_dead_end) score -= 500;

    // Future route pressure: cells on likely future box deliveries or compatible
    // agent access paths are legal fallback positions, but they are poor parking
    // targets because they will probably have to be cleared again.
    const int route_visits = future_route_visits(p, level, state);
    if (route_visits == 0) score += 2500;
    else score -= route_visits * 6000;

    // Prefer storage farther from current boxes and goals as a final tie-breaker;
    // this separates otherwise identical corridor cells in long narrow maps.
    score += std::min(nearest_goal_or_box_distance(p, level, state), 12) * 50;

    // Dynamic plan-awareness: a parking cell that lies on one of the already
    // planned agent trajectories is still legal, but it is less desirable
    // because parking a blocker there is likely to force CBS-style repair to
    // add waits or detours for that agent.
    score -= agent_path_penalty(p, initial_agent_plans);

    return score;
}

std::vector<Position> ParkingCellAnalyzer::find_parking_cells(const Level& level, const State& state, const LevelAnalysis& analysis) const {
    static const std::vector<AgentPlan> kNoInitialAgentPlans;
    return find_parking_cells(level, state, analysis, kNoInitialAgentPlans);
}

std::vector<Position> ParkingCellAnalyzer::find_parking_cells(const Level& level,
                                                              const State& state,
                                                              const LevelAnalysis& analysis,
                                                              const std::vector<AgentPlan>& initial_agent_plans) const {
    // Score every statically free cell produced by LevelAnalyzer. The input list
    // already excludes walls, and score_parking_cell applies dynamic exclusions
    // such as current box occupancy.
    std::vector<std::pair<Position, int>> scored;
    scored.reserve(analysis.free_cells.size());
    for (const Position p : analysis.free_cells) {
        const int score = score_parking_cell(p, level, state, analysis, initial_agent_plans);
        // std::cerr << "pos: " << p.to_string() << " score: " << score << std::endl;
        if (score > 0) scored.emplace_back(p, score);
    }

    // Highest scores should be tried first by callers. Ties intentionally keep
    // the default sort behavior unspecified because all equal-scoring cells are
    // considered equally safe by this heuristic.
    std::sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) { return a.second > b.second; });

    // Return only positions. The numeric score remains available through
    // LevelAnalysis::CellInfo::parking_score when LevelAnalyzer populates it.
    std::vector<Position> result;
    result.reserve(scored.size());
    for (const auto& [p, _] : scored) result.push_back(p);

    return result;
}
