#include "tasks/TaskGenerator.hpp"

#include "analysis/LevelAnalyzer.hpp"
#include "hospital/BlockerResolver.hpp"

#include <set>
#include <sstream>

namespace {
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

bool has_route_overlap_risk(const Task& a, const Task& b) {
    const auto ra = coarse_route(a.box_pos, a.goal_pos);
    const auto rb = coarse_route(b.box_pos, b.goal_pos);
    for (std::size_t i = 0; i < ra.size(); ++i) {
        for (std::size_t j = 0; j < rb.size(); ++j) {
            if (ra[i] == rb[j]) return true;
            if (i + 1 < ra.size() && j + 1 < rb.size() && ra[i] == rb[j + 1] && ra[i + 1] == rb[j]) return true;
        }
    }
    return false;
}

int manhattan(const Position& a, const Position& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}
}

bool TaskGenerator::is_box_goal(char goal_symbol) noexcept {
    return goal_symbol >= 'A' && goal_symbol <= 'Z';
}

bool TaskGenerator::can_agent_move_box(const Level& level, int agent_id, char box_id) {
    if (agent_id < 0 || agent_id >= static_cast<int>(level.agent_colors.size())) {
        return false;
    }
    if (box_id < 'A' || box_id > 'Z') {
        return false;
    }

    return level.agent_colors[agent_id] == level.box_colors[box_id - 'A'];
}

std::vector<Task> TaskGenerator::generate_delivery_tasks(const Level& level,
                                                         const State& state) {
    skip_reasons_.clear();
    std::vector<Task> tasks;
    int next_task_id = 0;
    std::set<std::tuple<char, int, int>> seen;

    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const char goal = level.goal_at(r, c);
            if (!is_box_goal(goal)) continue;
            if (state.box_at(r, c) == goal) continue;

            Position box_pos{-1, -1};
            for (int br = 0; br < state.rows; ++br) {
                for (int bc = 0; bc < state.cols; ++bc) {
                    if (state.box_at(br, bc) == goal) {
                        box_pos = Position{br, bc};
                        break;
                    }
                }
                if (box_pos.row != -1) break;
            }

            if (box_pos.row == -1) {
                skip_reasons_.push_back("skip goal " + std::string(1, goal) + " at (" + std::to_string(r) + "," + std::to_string(c) + "): missing box");
                continue;
            }

            int assigned_agent = -1;
            int best_dist = 1e9;
            for (int agent_id = 0; agent_id < state.num_agents(); ++agent_id) {
                if (!can_agent_move_box(level, agent_id, goal)) continue;
                const Position ap = state.agent_positions[agent_id];
                const int d = std::abs(ap.row - box_pos.row) + std::abs(ap.col - box_pos.col);
                if (d < best_dist) {
                    best_dist = d;
                    assigned_agent = agent_id;
                }
            }
            if (assigned_agent == -1) {
                skip_reasons_.push_back("skip goal " + std::string(1, goal) + " at (" + std::to_string(r) + "," + std::to_string(c) + "): no compatible agent");
                continue;
            }

            if (!seen.emplace(goal, r, c).second) continue;
            Task task;
            task.type = TaskType::DeliverBoxToGoal;
            task.task_id = next_task_id++;
            task.agent_id = assigned_agent;
            task.box_id = goal;
            task.box_pos = box_pos;
            task.goal_symbol = goal;
            task.goal_pos = Position{r, c};
            tasks.push_back(task);
        }
    }

    LevelAnalyzer analyzer;
    const LevelAnalysis analysis = analyzer.analyze(level, state);
    BlockerResolver blocker_resolver;
    std::vector<Task> blocker_tasks = blocker_resolver.generate_blocker_tasks(level, state, analysis, next_task_id);

    // If two delivery tasks are likely to conflict on overlapping corridors/routes,
    // add an explicit temporary relocation task for one of the boxes so that
    // scheduling can make progress instead of retrying the same pair.
    std::set<char> blocker_box_ids;
    for (const Task& bt : blocker_tasks) blocker_box_ids.insert(bt.box_id);
    for (std::size_t i = 0; i < tasks.size(); ++i) {
        for (std::size_t j = i + 1; j < tasks.size(); ++j) {
            const Task& a = tasks[i];
            const Task& b = tasks[j];
            if (a.type != TaskType::DeliverBoxToGoal || b.type != TaskType::DeliverBoxToGoal) continue;
            if (!has_route_overlap_risk(a, b)) continue;

            const Task& blocked = (a.task_id < b.task_id) ? b : a;
            if (blocker_box_ids.count(blocked.box_id)) continue;
            if (analysis.parking_cells.empty()) continue;

            Position best_park = analysis.parking_cells.front();
            int best_score = -1e9;
            for (const Position& p : analysis.parking_cells) {
                if (p == blocked.goal_pos) continue;
                int score = -manhattan(blocked.box_pos, p);
                const auto& cell = analysis.at(p);
                if (cell.is_chokepoint) score -= 25;
                if (cell.is_corridor) score -= 10;
                score += cell.parking_score;
                if (score > best_score) {
                    best_score = score;
                    best_park = p;
                }
            }

            Task t;
            t.type = TaskType::MoveBlockingBoxToParking;
            t.task_id = next_task_id++;
            t.agent_id = blocked.agent_id;
            t.box_id = blocked.box_id;
            t.box_pos = blocked.box_pos;
            t.parking_pos = best_park;
            t.goal_pos = best_park;
            blocker_tasks.push_back(t);
            blocker_box_ids.insert(blocked.box_id);
            skip_reasons_.push_back("added relocation task for box " + std::string(1, blocked.box_id) +
                                    " due to route overlap risk");
        }
    }

    tasks.insert(tasks.end(), blocker_tasks.begin(), blocker_tasks.end());

    return tasks;
}
