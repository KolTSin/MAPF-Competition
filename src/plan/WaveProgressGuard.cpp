#include "plan/WaveProgressGuard.hpp"

#include "actions/ActionSemantics.hpp"

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <queue>
#include <sstream>
#include <utility>
#include <vector>

namespace {
std::string pos_key(Position p) {
    return std::to_string(p.row) + "," + std::to_string(p.col);
}

std::string box_layout_signature(const State& state) {
    std::vector<std::string> boxes;
    boxes.reserve(static_cast<std::size_t>(state.rows * state.cols));
    for (int r = 0; r < state.rows; ++r) {
        for (int c = 0; c < state.cols; ++c) {
            const char box = state.box_at(r, c);
            if (box == '\0') continue;
            boxes.push_back(std::string(1, box) + "@" + pos_key({r, c}));
        }
    }
    std::sort(boxes.begin(), boxes.end());

    std::string signature;
    for (const std::string& box : boxes) signature += box + ";";
    return signature;
}
}

void WaveProgressGuard::reset(const Level& level, const State& initial_state) {
    (void)level;
    seen_worlds_.clear();
    trajectory_best_distance_.clear();
    remember_accepted(initial_state);
}

void WaveProgressGuard::remember_accepted(const State& state) {
    seen_worlds_.insert(world_signature(state));
}

int WaveProgressGuard::goals_completed(const Level& level, const State& state) {
    int done = 0;
    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const char goal = level.goal_at(r, c);
            if (goal >= 'A' && goal <= 'Z' && state.box_at(r, c) == goal) ++done;
        }
    }
    return done;
}

std::string WaveProgressGuard::world_signature(const State& state) {
    std::string signature;
    signature.reserve(static_cast<std::size_t>(state.rows * state.cols + state.num_agents() * 8));
    for (const Position& agent : state.agent_positions) {
        signature += "a" + pos_key(agent) + ";";
    }
    signature += "|" + box_layout_signature(state);
    return signature;
}

int WaveProgressGuard::nearest_goal_distance(const Level& level, char box, const State& state) {
    std::queue<Position> q;
    std::vector<int> dist(static_cast<std::size_t>(level.rows * level.cols), -1);
    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            if (state.box_at(r, c) != box) continue;
            dist[static_cast<std::size_t>(level.index(r, c))] = 0;
            q.push({r, c});
        }
    }
    if (q.empty()) return std::numeric_limits<int>::max() / 4;

    while (!q.empty()) {
        const Position cur = q.front();
        q.pop();
        const int cur_dist = dist[static_cast<std::size_t>(level.index(cur.row, cur.col))];
        if (level.goal_at(cur.row, cur.col) == box) return cur_dist;
        constexpr int dr[4] = {-1, 1, 0, 0};
        constexpr int dc[4] = {0, 0, -1, 1};
        for (int i = 0; i < 4; ++i) {
            const int nr = cur.row + dr[i];
            const int nc = cur.col + dc[i];
            if (!level.in_bounds(nr, nc) || level.is_wall(nr, nc)) continue;
            const int idx = level.index(nr, nc);
            if (dist[static_cast<std::size_t>(idx)] != -1) continue;
            dist[static_cast<std::size_t>(idx)] = cur_dist + 1;
            q.push({nr, nc});
        }
    }
    return std::numeric_limits<int>::max() / 4;
}

WaveProgressDecision WaveProgressGuard::assess(const Level& level,
                                               const State& before,
                                               const Plan& wave,
                                               int best_goals_completed) {
    WaveProgressDecision decision;
    decision.after = before;
    const int goals_before = goals_completed(level, before);
    const std::string boxes_before = box_layout_signature(before);

    struct LastMove { Position from; Position to; };
    std::unordered_map<std::string, LastMove> last_move_by_box;
    std::vector<std::pair<std::string, int>> observed_trajectories;

    State simulated = before;
    for (const JointAction& step : wave.steps) {
        for (int agent = 0; agent < simulated.num_agents() && agent < static_cast<int>(step.actions.size()); ++agent) {
            const ActionEffect effect = ActionSemantics::compute_effect(simulated.agent_positions[agent], step.actions[static_cast<std::size_t>(agent)]);
            simulated.agent_positions[agent] = effect.agent_to;
            if (!effect.moves_box || !simulated.in_bounds(effect.box_from.row, effect.box_from.col) || !simulated.in_bounds(effect.box_to.row, effect.box_to.col)) {
                continue;
            }

            const char moved = simulated.box_at(effect.box_from.row, effect.box_from.col);
            if (moved == '\0') continue;

            const std::string box_key = std::string(1, moved);
            const auto prev = last_move_by_box.find(box_key);
            if (prev != last_move_by_box.end() && prev->second.from == effect.box_to && prev->second.to == effect.box_from) {
                decision.reason = "cycle_detected: immediate_box_oscillation " + box_key + " " + pos_key(effect.box_from) + "->" + pos_key(effect.box_to);
            }

            const int before_dist = nearest_goal_distance(level, moved, simulated);
            simulated.set_box(effect.box_from.row, effect.box_from.col, '\0');
            simulated.set_box(effect.box_to.row, effect.box_to.col, moved);
            const int after_dist = nearest_goal_distance(level, moved, simulated);

            const std::string trajectory_key = box_key + ":" + pos_key(effect.box_from) + "->" + pos_key(effect.box_to);
            const int best_dist = std::min(before_dist, after_dist);
            const auto old_best = trajectory_best_distance_.find(trajectory_key);
            if (old_best != trajectory_best_distance_.end() && after_dist >= old_best->second && goals_completed(level, simulated) <= goals_before) {
                decision.accept = false;
                decision.reason = "cycle_detected: repeated_task_local_trajectory " + trajectory_key;
            }
            observed_trajectories.emplace_back(trajectory_key, best_dist);

            last_move_by_box[box_key] = LastMove{effect.box_from, effect.box_to};
        }
    }

    decision.after = simulated;
    decision.goals_after = goals_completed(level, simulated);
    decision.completed_new_goal = decision.goals_after > goals_before;
    decision.moved_any_box = box_layout_signature(simulated) != boxes_before;

    if (!seen_worlds_.empty() && seen_worlds_.find(world_signature(simulated)) != seen_worlds_.end()) {
        decision.accept = false;
        if (decision.reason.empty()) decision.reason = "cycle_detected: repeated_world";
    } else if (!decision.completed_new_goal && decision.goals_after < best_goals_completed) {
        decision.accept = false;
        decision.reason = "stagnation_detected: regressed_goal_progress";
    }

    if (!decision.accept && decision.reason.empty()) decision.reason = "cycle_detected";
    if (decision.accept) {
        for (const auto& [key, distance] : observed_trajectories) {
            auto [it, inserted] = trajectory_best_distance_.emplace(key, distance);
            if (!inserted) it->second = std::min(it->second, distance);
        }
    }
    return decision;
}
