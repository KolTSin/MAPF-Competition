#include "tasks/TaskGenerator.hpp"

#include "analysis/LevelAnalyzer.hpp"
#include "hospital/BlockerResolver.hpp"

#include <set>
#include <sstream>


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

    tasks.insert(tasks.end(), blocker_tasks.begin(), blocker_tasks.end());

    return tasks;
}
