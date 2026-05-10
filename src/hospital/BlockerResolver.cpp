#include "hospital/BlockerResolver.hpp"
#include "hospital/BoxTransportPlanner.hpp"
#include <algorithm>
#include <array>
#include <limits>
#include <queue>
#include <set>
#include <vector>

namespace {
// Four-neighbor movement deltas used by all grid searches in this file. Each
// helper treats the hospital as an unweighted grid: input positions are cells,
// and output paths/reachability answers move one row/column at a time.
constexpr int DR[4] = {-1, 1, 0, 0};
constexpr int DC[4] = {0, 0, 1, -1};

// Lightweight distance heuristic for tie-breaking. It ignores walls and boxes,
// so callers only use it for choosing the closest reasonable agent/cell, never
// as proof that a route exists.
int manhattan(const Position& a, const Position& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

// Answers "would there be any box at p after moving one candidate blocker?"
// Input: the real state plus a hypothetical relocation from moved_from to
// parked_at. Output: true if p is occupied in that simulated state.
bool simulated_box_at(const State& state, Position p, Position moved_from, Position parked_at) {
    if (p == moved_from) return false;
    if (p == parked_at) return true;
    return state.has_box(p.row, p.col);
}

// Same simulation as simulated_box_at, but returns the occupying box letter.
// This lets reachability checks reason about the moved box specifically rather
// than only knowing that some box occupies a cell.
char simulated_box_char_at(const State& state, Position p, char moved_box, Position moved_from, Position parked_at) {
    if (p == moved_from) return '\0';
    if (p == parked_at) return moved_box;
    return state.box_at(p.row, p.col);
}

// Breadth-first search from an agent cell to any cell adjacent to a target box.
// Inputs:
// - start: the agent position to search from.
// - box: the box cell the agent must stand next to.
// - moved_from/parked_at: optional one-box relocation to simulate.
// - boxes_block: when true, boxes are obstacles; when false, only walls block.
// Output: true when an adjacent cell is reachable; out_path, when supplied, is
// the start-to-adjacent-cell route used to inspect what boxes would block it.
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

// Convenience wrapper for the common case where no hypothetical relocation is
// being tested. The input/output contract matches path_to_box_neighbor_simulated.
bool path_to_box_neighbor(const Level& level, const State& state, Position start, Position box, bool boxes_block, std::vector<Position>* out_path) {
    return path_to_box_neighbor_simulated(level, state, start, box, Position{-1, -1}, Position{-1, -1}, boxes_block, out_path);
}

// Checks whether a box could theoretically travel from start to goal if all
// non-endpoint boxes were treated as obstacles in the simulated state. This is
// intentionally conservative: a parking cell is rejected when it disconnects any
// still-unsatisfied box from its goal even before detailed push planning runs.
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

// Locates a box in the simulated state. If the requested box is the one being
// moved, the output is parked_at; otherwise the output is its current state cell
// unless that cell is exactly the vacated moved_from position.
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

// Validates a candidate parking cell before emitting a relocation task. Inputs
// describe one hypothetical blocker move; output true means parking there does
// not occupy a goal, chokepoint, articulation point, forbidden delivery target,
// or disconnect any remaining goal box from a compatible agent and its goal.
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
        if (cell.is_intersection) return false;
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

// Produces a cheap row-then-column route from a box to its goal. This is not a
// final Sokoban plan; it is a coarse signal used to spot obvious boxes sitting
// between an unsatisfied box and its target. Output includes both endpoints.
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

// Returns true when p lies on the coarse route of any still-unsatisfied goal.
// Parking on such a cell is allowed only as a fallback, but it receives a score
// penalty because it may create work for a later delivery.
bool on_future_coarse_route(const Level& level, const State& state, Position p) {
    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const char goal = level.goal_at(r, c);
            if (goal < 'A' || goal > 'Z') continue;
            if (state.box_at(r, c) == goal) continue;
            // Locate the current source box for this goal. If it exists, its
            // coarse path marks cells where parking would likely cause future work.
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

// Scores a parking candidate before full validation. Higher output means the
// cell is more attractive: prefer analyzer-approved parking cells close to the
// blocker, and strongly penalize goals, corridors, chokepoints, articulation
// points, or cells that future coarse deliveries likely need.
int base_parking_score(const Level& level, const State& state, const LevelAnalysis& analysis, Position box_pos, Position p) {
    int score = analysis.at(p).parking_score - manhattan(box_pos, p);
    score += (level.goal_at(p.row, p.col) == '\0') ? 20 : -1000;
    if (analysis.rows == state.rows && analysis.cols == state.cols && !analysis.cells.empty()) {
        const auto& cell = analysis.at(p);
        if (cell.is_dead_end) score += 12000;
        if (cell.is_intersection) score -= 30000;
        if (!cell.is_dead_end) {
            if (cell.is_chokepoint) score -= 5000;
            if (cell.is_articulation) score -= 5000;
        }
        if (cell.is_corridor) score -= 1000;
    }
    if (on_future_coarse_route(level, state, p)) score -= 500;
    return score;
}

// Chooses the relocation target for one blocker. Inputs identify the moved box,
// its current position, and a forbidden cell such as the delivery goal being
// cleared. Output is the best reachability-preserving parking cell; if none pass
// the strict checks, the best non-forbidden fallback is returned so planning can
// still attempt recovery instead of dropping the blocker task completely.
bool box_transport_feasible(const Level& level, const State& state, char moved_box, Position box_pos, Position parked_at, int agent_id) {
    if (agent_id < 0 || agent_id >= state.num_agents()) return true;

    Task probe;
    probe.type = TaskType::MoveBlockingBoxToParking;
    probe.task_id = -1;
    probe.agent_id = agent_id;
    probe.box_id = moved_box;
    probe.box_pos = box_pos;
    probe.parking_pos = parked_at;
    probe.goal_pos = parked_at;

    BoxTransportPlanner planner;
    return planner.plan(level, state, probe).success;
}

Position best_parking_for(const Level& level, const State& state, const LevelAnalysis& analysis, char moved_box, Position box_pos, Position forbidden, int* out_score = nullptr, int agent_id = -1) {
    std::vector<Position> candidates = analysis.parking_cells;
    for (const Position& p : analysis.free_cells) {
        if (std::find(candidates.begin(), candidates.end(), p) != candidates.end()) continue;
        if (!level.in_bounds(p.row, p.col) || level.is_wall(p.row, p.col)) continue;
        if (state.has_box(p.row, p.col) && !(p == box_pos)) continue;
        if (level.goal_at(p.row, p.col) != '\0') continue;
        candidates.push_back(p);
    }

    std::sort(candidates.begin(), candidates.end(), [&](Position a, Position b) {
        return base_parking_score(level, state, analysis, box_pos, a) >
               base_parking_score(level, state, analysis, box_pos, b);
    });

    Position best = candidates.empty() ? Position{-1, -1} : candidates.front();
    int best_score = std::numeric_limits<int>::min();
    Position fallback = best;
    int fallback_score = std::numeric_limits<int>::min();
    for (const Position& p : candidates) {
        const int score = base_parking_score(level, state, analysis, box_pos, p);
        if (score > fallback_score && p != box_pos && p != forbidden) { fallback_score = score; fallback = p; }
        if (!parking_preserves_future_reachability(level, state, analysis, moved_box, box_pos, p, forbidden)) continue;
        if (!box_transport_feasible(level, state, moved_box, box_pos, p, agent_id)) continue;
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

// Builds all blocker-relocation tasks for the current snapshot. The resolver is
// deliberately heuristic: it does not move boxes itself; it emits intuitive task
// records for later schedulers/planners to execute and validate.
std::vector<Task> BlockerResolver::generate_blocker_tasks(const Level& level,
                                                          const State& state,
                                                          const LevelAnalysis& analysis,
                                                          int& next_task_id) const {
    std::vector<Task> tasks;

    // Without analyzer-provided free cells, this resolver has no output target for
    // relocation tasks, so the correct output is an empty task list. Low-scoring
    // free cells may still be considered as a fallback by best_parking_for().
    if (analysis.free_cells.empty()) return tasks;

    // Track box letters already converted into relocation tasks. Each box can
    // appear on multiple coarse/access routes, but emitting duplicate tasks for
    // the same physical box would confuse task ordering downstream.
    std::set<char> already_selected;

    // First classify letters that are still needed for unsatisfied goals. Boxes
    // with these letters should normally remain delivery candidates rather than
    // being treated as generic clutter in the first pass.
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

    // Pass 1: park obvious clutter boxes.
    // Input: every current box cell. Output: relocation tasks for boxes that are
    // not already on a goal and are not needed by any unsatisfied goal letter.
    // These tasks reduce congestion before more goal-specific blockers are added.
    for (int r = 0; r < state.rows; ++r) {
        for (int c = 0; c < state.cols; ++c) {
            const char b = state.box_at(r, c);
            if (b == '\0') continue;
            if (level.goal_at(r, c) != '\0') continue;
            if (needed_for_goal[static_cast<std::size_t>(b - 'A')]) continue;

            // The task output contains the blocker identity, its current input
            // position, the chosen parking output position, and an initial agent
            // assignment for a color-compatible mover.
            Task t;
            t.type = TaskType::MoveBlockingBoxToParking;
            t.task_id = next_task_id++;
            t.box_id = b;
            t.box_pos = Position{r, c};
            // Prefer the nearest compatible agent by Manhattan distance so parking
            // selection can cheaply reject cells this mover cannot actually reach.
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
            int best_score = std::numeric_limits<int>::min();
            Position best_park = best_parking_for(level, state, analysis, b, t.box_pos, Position{-1, -1}, &best_score, t.agent_id);
            t.parking_pos = best_park;
            t.goal_pos = t.parking_pos;
            t.priority = best_score;
            tasks.push_back(t);
            already_selected.insert(b);
        }
    }

    // Pass 2: clear agent-access blockers.
    // For each unsatisfied goal, find the matching active box and the closest
    // compatible agent. If walls allow the agent to reach the box only when boxes
    // are ignored, the path returned by the relaxed search identifies which box
    // currently gates the approach corridor. The output is one relocation task
    // for the first such gate, annotated with unblocks_box_id so dependency
    // building can run this task before the affected delivery.
    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const char goal = level.goal_at(r, c);
            if (goal < 'A' || goal > 'Z') continue;
            if (state.box_at(r, c) == goal) continue;

            // Locate the current source box for this goal letter. If no matching
            // box exists, there is no concrete delivery route to unblock.
            Position active_box{-1, -1};
            for (int br = 0; br < state.rows; ++br) {
                for (int bc = 0; bc < state.cols; ++bc) {
                    if (state.box_at(br, bc) == goal) { active_box = Position{br, bc}; break; }
                }
                if (active_box.row != -1) break;
            }
            if (active_box.row == -1) continue;

            // Pick the closest agent that is allowed to move this goal letter.
            // The selected agent is used only to test whether a box blocks access
            // to the delivery box; final scheduling may still reorder tasks.
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

            // If the normal search succeeds with boxes blocking, access is already
            // clear and no relocation task is needed for this goal's approach.
            if (path_to_box_neighbor(level, state, agent_pos, active_box, true, nullptr)) continue;

            // Retry while ignoring boxes. A successful relaxed path means walls are
            // not the problem; boxes on this returned path are candidate blockers.
            std::vector<Position> passable_path;
            if (!path_to_box_neighbor(level, state, agent_pos, active_box, false, &passable_path)) continue;
            for (const Position& p : passable_path) {
                const char blocker = state.box_at(p.row, p.col);
                if (blocker == '\0' || blocker == goal || already_selected.count(blocker)) continue;

                // Do not pull boxes off goal cells in this heuristic pass. A goal
                // cell may be a deliberate final placement, so moving it requires
                // stronger reasoning than simple access clearing.
                if (level.goal_at(p.row, p.col) != '\0') continue;

                // Assign the relocation to the nearest agent with the blocker box's
                // color; if no such agent exists, stop inspecting this route because
                // this blocker cannot be moved by the current team.
                int blocker_agent = -1;
                int blocker_best_dist = std::numeric_limits<int>::max();
                const Color blocker_color = level.box_colors[static_cast<std::size_t>(blocker - 'A')];
                for (int a = 0; a < state.num_agents(); ++a) {
                    if (level.agent_colors[static_cast<std::size_t>(a)] != blocker_color) continue;
                    const int d = manhattan(state.agent_positions[static_cast<std::size_t>(a)], p);
                    if (d < blocker_best_dist) { blocker_best_dist = d; blocker_agent = a; }
                }
                if (blocker_agent < 0) break;

                // Emit a high-priority access-clearing task. The parking target is
                // chosen while forbidding the affected goal cell, and unblocks_box_id
                // records the delivery letter that should wait for this relocation.
                Task t;
                t.type = TaskType::MoveBlockingBoxToParking;
                t.task_id = next_task_id++;
                t.box_id = blocker;
                t.box_pos = p;
                int selected_score = 0;
                t.parking_pos = best_parking_for(level, state, analysis, blocker, p, Position{r, c}, &selected_score, blocker_agent);
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

    // Pass 3: clear boxes directly on coarse delivery routes.
    // For every unsatisfied goal, build a cheap row-then-column path from the
    // active box to that goal. Any other box on that path is an obvious route
    // blocker. Output tasks from this pass are lower priority than access gates
    // because they are based on a coarse route rather than observed agent access.
    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const char goal = level.goal_at(r, c);
            if (goal < 'A' || goal > 'Z') continue;
            if (state.box_at(r, c) == goal) continue;

            // Find the box that still needs to be delivered to this goal. The
            // coarse route is meaningful only if such a source box exists.
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

                // Build a route-blocker task: move box b from q to a selected
                // parking cell and mark that it unblocks the current goal letter.
                Task t;
                t.type = TaskType::MoveBlockingBoxToParking;
                t.task_id = next_task_id++;
                t.box_id = b;
                t.box_pos = q;
                t.unblocks_box_id = goal;
                t.debug_label = "route_blocker_for_" + std::string(1, goal);

                // Route blockers are assigned the nearest color-compatible agent,
                // matching the convention used for clutter and access blockers.
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
                int best_score = 0;
                const Position best_park = best_parking_for(level, state, analysis, b, q, Position{r, c}, &best_score, best_agent);
                t.parking_pos = best_park;
                t.goal_pos = best_park;
                t.priority = best_score + 20;
                tasks.push_back(t);
                already_selected.insert(b);
            }
        }
    }
    return tasks;
}
