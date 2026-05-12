#include "analysis/LevelAnalyzer.hpp"
#include "analysis/ParkingCellAnalyzer.hpp"
#include <algorithm>
#include <limits>
#include <queue>

namespace {
// Shared four-neighbor movement offsets. Each index describes one cardinal
// step: up, down, left, or right. All topology in this analyzer is based on
// the same movement model used by agents and boxes.
constexpr int DR[4] = {-1, 1, 0, 0};
constexpr int DC[4] = {0, 0, -1, 1};
// Diagonal directions: NW, NE, SW, SE
static constexpr int DDR[4] = {-1, -1, 1, 1};
static constexpr int DDC[4] = {-1, 1, -1, 1};

int manhattan(Position a, Position b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

Position find_box(const State& state, char box) {
    for (int r = 0; r < state.rows; ++r) {
        for (int c = 0; c < state.cols; ++c) {
            if (state.box_at(r, c) == box) return Position{r, c};
        }
    }
    return Position{-1, -1};
}

std::vector<Position> shortest_path_ignoring_boxes(const Level& level, Position start, Position goal, const PlanningDeadline* deadline = nullptr) {
    if (!level.in_bounds(start.row, start.col) || !level.in_bounds(goal.row, goal.col)) return {};
    if (level.is_wall(start.row, start.col) || level.is_wall(goal.row, goal.col)) return {};

    std::vector<bool> seen(static_cast<std::size_t>(level.rows * level.cols), false);
    std::vector<Position> parent(static_cast<std::size_t>(level.rows * level.cols), Position{-1, -1});
    std::queue<Position> q;
    q.push(start);
    seen[static_cast<std::size_t>(level.index(start.row, start.col))] = true;

    while (!q.empty()) {
        if (deadline != nullptr && deadline->expired()) return {};
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

std::vector<Position> shortest_path_to_box_neighbor(const Level& level, Position start, Position box, const PlanningDeadline* deadline = nullptr) {
    if (!level.in_bounds(start.row, start.col) || level.is_wall(start.row, start.col)) return {};

    std::vector<bool> seen(static_cast<std::size_t>(level.rows * level.cols), false);
    std::vector<Position> parent(static_cast<std::size_t>(level.rows * level.cols), Position{-1, -1});
    std::queue<Position> q;
    q.push(start);
    seen[static_cast<std::size_t>(level.index(start.row, start.col))] = true;

    Position target{-1, -1};
    while (!q.empty() && target.row == -1) {
        if (deadline != nullptr && deadline->expired()) return {};
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

void add_path_visits(LevelAnalysis& analysis, const Level& level, const std::vector<Position>& path) {
    for (const Position p : path) {
        if (!level.in_bounds(p.row, p.col) || level.is_wall(p.row, p.col)) continue;
        ++analysis.at(p).future_route_visits;
    }
}

void compute_initial_route_pressure(LevelAnalysis& analysis, const Level& level, const State& state, const PlanningDeadline* deadline = nullptr) {
    for (Position p : analysis.free_cells) {
        if (deadline != nullptr && deadline->expired()) return;
        analysis.at(p).future_route_visits = 0;
    }

    for (int r = 0; r < level.rows; ++r) {
        if (deadline != nullptr && deadline->expired()) return;
        for (int c = 0; c < level.cols; ++c) {
            if (deadline != nullptr && deadline->expired()) return;
            const char goal = level.goal_at(r, c);
            if (goal < 'A' || goal > 'Z') continue;
            if (state.box_at(r, c) == goal) continue;

            const Position box = find_box(state, goal);
            if (box.row == -1) continue;
            add_path_visits(analysis, level, shortest_path_ignoring_boxes(level, box, Position{r, c}, deadline));

            const Color box_color = level.box_colors[static_cast<std::size_t>(goal - 'A')];
            for (int agent = 0; agent < state.num_agents(); ++agent) {
                if (level.agent_colors[static_cast<std::size_t>(agent)] != box_color) continue;
                add_path_visits(analysis, level, shortest_path_to_box_neighbor(level, state.agent_positions[static_cast<std::size_t>(agent)], box, deadline));
            }
        }
    }
}

void update_nearest_goal_or_box_distance(LevelAnalysis& analysis, const Level& level, const State& state) {
    std::vector<Position> targets;
    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const char goal = level.goal_at(r, c);
            if (goal != '\0' && goal != ' ') targets.push_back(Position{r, c});
            if (state.has_box(r, c)) targets.push_back(Position{r, c});
        }
    }

    for (Position p : analysis.free_cells) {
        int best = std::numeric_limits<int>::max();
        for (const Position target : targets) {
            best = std::min(best, manhattan(p, target));
        }
        analysis.at(p).nearest_goal_or_box_distance = (best == std::numeric_limits<int>::max()) ? 0 : best;
    }
}


void update_agent_path_visits(LevelAnalysis& analysis, const std::vector<AgentPlan>& initial_agent_plans) {
    for (Position p : analysis.free_cells) analysis.at(p).agent_path_visits = 0;
    for (const AgentPlan& plan : initial_agent_plans) {
        for (const Position& p : plan.positions) {
            if (p.row < 0 || p.col < 0 || p.row >= analysis.rows || p.col >= analysis.cols) continue;
            CellInfo& cell = analysis.at(p);
            if (!cell.is_free_static) continue;
            ++cell.agent_path_visits;
        }
    }
}

void refresh_dynamic_parking(LevelAnalysis& analysis,
                             const Level& level,
                             const State& state,
                             const std::vector<AgentPlan>& initial_agent_plans,
                             const PlanningDeadline* deadline = nullptr) {
    if (deadline != nullptr && deadline->expired()) return;
    update_nearest_goal_or_box_distance(analysis, level, state);
    if (deadline != nullptr && deadline->expired()) return;
    update_agent_path_visits(analysis, initial_agent_plans);

    ParkingCellAnalyzer parking;
    analysis.parking_cells = parking.find_parking_cells(level, state, analysis, initial_agent_plans);
    for (const Position p : analysis.free_cells) {
        if (deadline != nullptr && deadline->expired()) return;
        analysis.at(p).parking_score = parking.score_parking_cell(p, level, state, analysis, initial_agent_plans);
    }
}
}

LevelAnalysis LevelAnalyzer::analyze(const Level& level, const State& state) const {
    static const std::vector<AgentPlan> kNoInitialAgentPlans;
    return analyze(level, state, kNoInitialAgentPlans);
}

LevelAnalysis LevelAnalyzer::analyze(const Level& level, const State& state, const std::vector<AgentPlan>& initial_agent_plans) const {
    PlanningDeadline no_deadline;
    return analyze(level, state, initial_agent_plans, no_deadline);
}

LevelAnalysis LevelAnalyzer::analyze(const Level& level, const State& state, const std::vector<AgentPlan>& initial_agent_plans, const PlanningDeadline& deadline) const {
    // Allocate a grid-shaped result. The cells vector has one CellInfo for
    // every coordinate, including walls; wall entries keep their default flags
    // while traversable entries are filled in by the scan below.
    LevelAnalysis analysis;
    analysis.rows = level.rows;
    analysis.cols = level.cols;
    analysis.cells.resize(static_cast<std::size_t>(analysis.rows * analysis.cols));

    // Pass 1: classify each non-wall cell using only local static geometry.
    //
    // Input for each coordinate: wall/goal data from Level and the four static
    // neighbors around the coordinate.
    // Output for each free cell: CellInfo flags for free space, degree,
    // corridor/dead-end/room shape, goal type, plus an entry in free_cells.
    for (int r = 0; r < level.rows; ++r) {
        if (deadline.expired()) return analysis;
        for (int c = 0; c < level.cols; ++c) {
            if (deadline.expired()) return analysis;
            Position p{r, c};
            if (level.is_wall(r, c)) continue;

            CellInfo& info = analysis.at(p);
            info.is_free_static = true;
            analysis.free_cells.push_back(p);

            // Degree is the number of traversable cardinal neighbors. It gives
            // a cheap, intuitive local shape label:
            //   0-1 neighbors: dead end
            //   2 neighbors: corridor-like passage
            //   3-4 neighbors: room/intersection with maneuvering space
            int degree = 0;
            int blocked_diagonals = 0;

            // Cardinal openness flags.
            // DR/DC order is currently: north, south, west, east.
            bool north_free = false;
            bool south_free = false;
            bool west_free = false;
            bool east_free = false;

            // Count free cardinal neighbors.
            for (int i = 0; i < 4; ++i) {
                const int nr = r + DR[i];
                const int nc = c + DC[i];

                const bool free =
                    level.in_bounds(nr, nc) &&
                    !level.is_wall(nr, nc);

                if (!free) continue;

                ++degree;

                if (DR[i] == -1 && DC[i] == 0) north_free = true;
                if (DR[i] ==  1 && DC[i] == 0) south_free = true;
                if (DR[i] ==  0 && DC[i] == -1) west_free = true;
                if (DR[i] ==  0 && DC[i] ==  1) east_free = true;
            }

            // Count blocked diagonals.
            // A diagonal is "not free" if it is out of bounds or a wall.
            for (int i = 0; i < 4; ++i) {
                const int nr = r + DDR[i];
                const int nc = c + DDC[i];

                if (!level.in_bounds(nr, nc) || level.is_wall(nr, nc)) {
                    ++blocked_diagonals;
                }
            }

            const bool vertical_corridor =
                north_free && south_free && !west_free && !east_free;

            const bool horizontal_corridor =
                west_free && east_free && !north_free && !south_free;

            const bool straight_corridor =
                degree == 2 && (vertical_corridor || horizontal_corridor);

            const bool corner =
                degree == 2 && !straight_corridor;

            const bool three_way_intersection = degree == 3 && blocked_diagonals >= 3;
            const bool four_way_intersection = degree == 4 && blocked_diagonals >= 3;

            // A straight corridor with multiple blocked diagonals remains a
            // corridor in the broad labels below; intersection flags are kept
            // for true three- and four-way connectors only.

            info.degree = degree;
            info.blocked_diagonals = blocked_diagonals;

            info.is_dead_end = degree <= 1;
            info.is_corner = corner;

            info.is_three_way_intersection = three_way_intersection;
            info.is_four_way_intersection = four_way_intersection;

            info.is_intersection =
                three_way_intersection ||
                four_way_intersection;

            // Keep your old broad labels.
            info.is_corridor = degree == 2;
            info.is_room = (degree - blocked_diagonals) >= 3;

            // Goal cells are annotated separately because parking or blocking a
            // goal can make a future task harder. The raw goal character also
            // tells callers whether the goal belongs to a box or an agent.
            const char g = level.goal_at(r, c);
            if (g != '\0' && g != ' ') {
                info.is_goal_cell = true;
                info.is_box_goal_cell = (g >= 'A' && g <= 'Z');
                info.is_agent_goal_cell = (g >= '0' && g <= '9');
            }
        }
    }

    // Pass 2: assign connected-component ids with breadth-first search.
    //
    // Input: the free_cells discovered above and static wall layout.
    // Output: component_id for every free cell. Cells with the same id are
    // mutually reachable when dynamic agents and boxes are ignored.
    int component = 0;
    std::vector<bool> visited(static_cast<std::size_t>(level.rows * level.cols), false);
    for (const Position start : analysis.free_cells) {
        if (deadline.expired()) return analysis;
        const int si = analysis.index(start);
        if (visited[static_cast<std::size_t>(si)]) continue;

        std::queue<Position> q;
        q.push(start);
        visited[static_cast<std::size_t>(si)] = true;

        while (!q.empty()) {
            if (deadline.expired()) return analysis;
            const Position cur = q.front();
            q.pop();
            analysis.at(cur).component_id = component;

            for (int i = 0; i < 4; ++i) {
                Position nxt{cur.row + DR[i], cur.col + DC[i]};
                if (!level.in_bounds(nxt.row, nxt.col) || level.is_wall(nxt.row, nxt.col)) continue;

                const int ni = analysis.index(nxt);
                if (visited[static_cast<std::size_t>(ni)]) continue;

                visited[static_cast<std::size_t>(ni)] = true;
                q.push(nxt);
            }
        }
        ++component;
    }

    // Pass 3: derive higher-level category lists from the per-cell labels.
    //
    // Chokepoints are currently approximated with low-degree geometry: dead
    // ends and corridor cells are places where an object is likely to block
    // flow. is_articulation mirrors this conservative chokepoint label so
    // downstream heuristics can treat it as a strong "do not park here" signal.
    for (const Position p : analysis.free_cells) {
        if (deadline.expired()) return analysis;
        CellInfo& info = analysis.at(p);
        info.is_chokepoint = info.is_dead_end || (info.is_corridor && info.degree <= 2);
        info.is_articulation = info.is_chokepoint;

        if (info.is_corridor) analysis.corridor_cells.push_back(p);
        if (info.is_room) analysis.room_cells.push_back(p);
        if (info.is_chokepoint) analysis.chokepoints.push_back(p);
    }

    // Pass 4: compute the expensive initial future-route pressure once.
    // Subsequent state updates reuse these values instead of rerunning BFS for
    // every parking candidate in every HTN wave.
    compute_initial_route_pressure(analysis, level, state, &deadline);

    // Pass 5: score and rank temporary parking cells using lightweight dynamic
    // occupancy and plan information.
    refresh_dynamic_parking(analysis, level, state, initial_agent_plans, &deadline);

    return analysis;
}

LevelAnalysis LevelAnalyzer::update(const Level& level, const State& state, const LevelAnalysis& base) const {
    static const std::vector<AgentPlan> kNoInitialAgentPlans;
    return update(level, state, base, kNoInitialAgentPlans);
}

LevelAnalysis LevelAnalyzer::update(const Level& level,
                                    const State& state,
                                    const LevelAnalysis& base,
                                    const std::vector<AgentPlan>& initial_agent_plans) const {
    PlanningDeadline no_deadline;
    return update(level, state, base, initial_agent_plans, no_deadline);
}

LevelAnalysis LevelAnalyzer::update(const Level& level,
                                    const State& state,
                                    const LevelAnalysis& base,
                                    const std::vector<AgentPlan>& initial_agent_plans,
                                    const PlanningDeadline& deadline) const {
    LevelAnalysis analysis = base;
    if (deadline.expired()) return analysis;
    compute_initial_route_pressure(analysis, level, state, &deadline);
    refresh_dynamic_parking(analysis, level, state, initial_agent_plans, &deadline);
    return analysis;
}
