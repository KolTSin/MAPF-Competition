#include "tasks/TaskGenerator.hpp"

#include "analysis/LevelAnalyzer.hpp"
#include "hospital/BlockerResolver.hpp"

#include <set>
#include <sstream>
#include <iostream>


bool TaskGenerator::is_box_goal(char goal_symbol) noexcept {
    // Sokoban-style levels use uppercase letters to mean "a box with this
    // letter should end here". Digits/lowercase/empty cells are goals for
    // other entities or just non-goal markers, so they do not become delivery
    // tasks.
    return goal_symbol >= 'A' && goal_symbol <= 'Z';
}

bool TaskGenerator::is_agent_goal(char goal_symbol) noexcept {
    return goal_symbol >= '0' && goal_symbol <= '9';
}

void TaskGenerator::append_agent_goal_tasks(const Level& level,
                                            const State& state,
                                            int& next_task_id,
                                            std::vector<Task>& tasks) {
    std::vector<int> final_task_dependencies;
    final_task_dependencies.reserve(tasks.size());
    for (const Task& task : tasks) {
        final_task_dependencies.push_back(task.task_id);
    }

    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const char goal = level.goal_at(r, c);
            if (!is_agent_goal(goal)) continue;

            const int agent_id = goal - '0';
            if (agent_id < 0 || agent_id >= state.num_agents()) {
                skip_reasons_.push_back("skip agent goal " + std::string(1, goal) + " at (" + std::to_string(r) + "," + std::to_string(c) + "): missing agent");
                continue;
            }

            const bool already_at_goal = state.agent_positions[agent_id] == Position{r, c};
            if (already_at_goal && final_task_dependencies.empty()) continue;

            Task task;
            task.type = TaskType::MoveAgentToGoal;
            task.task_id = next_task_id++;
            task.agent_id = agent_id;
            task.goal_symbol = goal;
            task.goal_pos = Position{r, c};
            task.box_pos = state.agent_positions[agent_id];
            task.dependencies = final_task_dependencies;
            tasks.push_back(task);
        }
    }
}

bool TaskGenerator::can_agent_move_box(const Level& level, int agent_id, char box_id) {
    // The input may come from generated levels or tests, so validate both sides
    // before indexing into the color arrays. A false result simply means this
    // agent cannot be assigned to this box.
    if (agent_id < 0 || agent_id >= static_cast<int>(level.agent_colors.size())) {
        return false;
    }
    if (box_id < 'A' || box_id > 'Z') {
        return false;
    }

    // Agents may only manipulate boxes of the same color. The box letter maps
    // directly to its color slot: 'A' -> 0, 'B' -> 1, and so on.
    return level.agent_colors[agent_id] == level.box_colors[box_id - 'A'];
}

std::vector<Task> TaskGenerator::generate_delivery_tasks(const Level& level,
                                                         const State& state) {
    static const std::vector<AgentPlan> kNoInitialAgentPlans;
    return generate_delivery_tasks(level, state, kNoInitialAgentPlans);
}

std::vector<Task> TaskGenerator::generate_delivery_tasks(const Level& level,
                                                         const State& state,
                                                         const std::vector<AgentPlan>& initial_agent_plans) {
    // Start each run with a clean output surface: callers receive only tasks
    // and skip reasons that describe the current level/state pair.
    skip_reasons_.clear();
    std::vector<Task> tasks;

    // Task ids are assigned sequentially across both delivery tasks and the
    // later blocker tasks so downstream solvers can refer to a stable id within
    // the returned vector.
    int next_task_id = 0;

    // Tracks goal coordinates already converted into a delivery task. The level
    // scan visits each cell once, but this guard documents and enforces the
    // output invariant: one delivery task per (box letter, goal row, goal col).
    std::set<std::tuple<char, int, int>> seen;

    // First phase: inspect every static goal cell and create direct delivery
    // tasks for goals that are not currently satisfied.
    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const char goal = level.goal_at(r, c);

            // Non-box goals do not require moving a box, and already-satisfied
            // goals should not create redundant work.
            if (!is_box_goal(goal)) continue;
            if (state.box_at(r, c) == goal) continue;

            // Locate the current position of the box that matches this goal
            // symbol. The output task needs both the source (`box_pos`) and the
            // destination (`goal_pos`) so the planner can compute a route.
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

            // A goal without a matching box cannot be delivered. Record the
            // reason rather than emitting an impossible task.
            if (box_pos.row == -1) {
                skip_reasons_.push_back("skip goal " + std::string(1, goal) + " at (" + std::to_string(r) + "," + std::to_string(c) + "): missing box");
                continue;
            }

            // Choose the closest compatible agent as a simple assignment
            // heuristic. Compatibility is color-based; distance is Manhattan
            // distance from the agent to the box's current position. The task's
            // output agent id is therefore intuitive: the nearest agent that is
            // legally allowed to push this box.
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
            // If no color-compatible agent exists, delivery is impossible in
            // the current level definition, so explain the skip to the caller.
            if (assigned_agent == -1) {
                skip_reasons_.push_back("skip goal " + std::string(1, goal) + " at (" + std::to_string(r) + "," + std::to_string(c) + "): no compatible agent");
                continue;
            }

            // Convert the validated input pieces into a complete delivery task:
            // who acts (`agent_id`), what they move (`box_id`/`box_pos`), and
            // where it must end (`goal_symbol`/`goal_pos`).
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

    // Second phase: augment direct deliveries with tasks that clear obstacles
    // detected by the level analysis. These blocker tasks share the same output
    // vector so solvers can plan deliveries and prerequisite clearing work from
    // one ordered task list.
    LevelAnalyzer analyzer;
    const LevelAnalysis analysis = analyzer.analyze(level, state, initial_agent_plans);
    BlockerResolver blocker_resolver;
    // std::cerr << "Parking cells on the level" << static_cast<int>(analysis.parking_cells.size()) << std::endl;
    // for (int i = 0; i < static_cast<int>(analysis.parking_cells.size()); i++) std::cerr << "parking cell: " 
    // << analysis.parking_cells[i].to_string() << std::endl;
    std::vector<Task> blocker_tasks = blocker_resolver.generate_blocker_tasks(level, state, analysis, next_task_id);

    tasks.insert(tasks.end(), blocker_tasks.begin(), blocker_tasks.end());

    // Final phase: once all generated box/blocker work has completed, agents
    // that have digit goals should return to their own goal cells.  These tasks
    // are intentionally appended last and depend on the earlier task ids so they
    // do not steal an agent away before its useful work is done.
    append_agent_goal_tasks(level, state, next_task_id, tasks);

    return tasks;
}
