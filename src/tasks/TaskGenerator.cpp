#include "tasks/TaskGenerator.hpp"

#include "analysis/LevelAnalyzer.hpp"
#include "hospital/BlockerResolver.hpp"

#include <algorithm>
#include <limits>
#include <set>
#include <sstream>
#include <iostream>

namespace {
int manhattan(Position a, Position b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

bool contains_position(const std::vector<Position>& positions, Position pos) {
    return std::find(positions.begin(), positions.end(), pos) != positions.end();
}

bool is_satisfied_box_goal(const Level& level, const State& state, Position pos, char box_id) {
    return level.goal_at(pos.row, pos.col) == box_id && state.box_at(pos.row, pos.col) == box_id;
}

Position choose_box_for_goal(const Level& level,
                             const State& state,
                             char box_id,
                             Position goal_pos,
                             const std::vector<Position>& assigned_boxes) {
    Position best{-1, -1};
    int best_score = std::numeric_limits<int>::max();

    for (int r = 0; r < state.rows; ++r) {
        for (int c = 0; c < state.cols; ++c) {
            const Position candidate{r, c};
            if (state.box_at(r, c) != box_id) continue;
            if (contains_position(assigned_boxes, candidate)) continue;

            // A same-letter box that is already on a matching goal is doing
            // useful work for another goal cell.  Only use it as an expensive
            // last resort if there are no unassigned off-goal boxes available.
            const int satisfied_goal_penalty = is_satisfied_box_goal(level, state, candidate, box_id) ? 100000 : 0;
            const int score = satisfied_goal_penalty + manhattan(candidate, goal_pos);
            if (score < best_score) {
                best_score = score;
                best = candidate;
            }
        }
    }

    return best;
}

std::size_t level_signature(const Level& level) {
    std::size_t h = static_cast<std::size_t>(level.rows) * 1315423911u + static_cast<std::size_t>(level.cols);
    const auto mix = [&h](std::size_t v) {
        h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    };

    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
                mix(level.is_wall(r, c) ? 1u : 0u);
            mix(static_cast<unsigned char>(level.goal_at(r, c)));
        }
    }
    for (const Color color : level.agent_colors) mix(static_cast<std::size_t>(color));
    for (const Color color : level.box_colors) mix(static_cast<std::size_t>(color));
    return h;
}
}

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
    PlanningDeadline no_deadline;
    return generate_delivery_tasks(level, state, initial_agent_plans, no_deadline);
}

std::vector<Task> TaskGenerator::generate_delivery_tasks(const Level& level,
                                                         const State& state,
                                                         const std::vector<AgentPlan>& initial_agent_plans,
                                                         const PlanningDeadline& deadline) {
    return generate_delivery_tasks(level, state, initial_agent_plans, deadline, TaskGenerationOptions{});
}

std::vector<Task> TaskGenerator::generate_delivery_tasks(const Level& level,
                                                         const State& state,
                                                         const std::vector<AgentPlan>& initial_agent_plans,
                                                         const PlanningDeadline& deadline,
                                                         const TaskGenerationOptions& options) {
    // Start each run with a clean output surface: callers receive only tasks
    // and skip reasons that describe the current level/state pair.
    skip_reasons_.clear();
    std::vector<Task> tasks;

    // Task ids are assigned sequentially across both delivery tasks and the
    // later blocker tasks so downstream solvers can refer to a stable id within
    // the returned vector.
    int next_task_id = 0;

    // Tracks goal coordinates and source boxes already converted into delivery
    // tasks.  Same-letter boxes are interchangeable in the server state, but the
    // task layer must still avoid assigning the same physical cell to multiple
    // unsatisfied goals in one wave.
    std::set<std::tuple<char, int, int>> seen;
    std::vector<Position> assigned_boxes;

    if (deadline.expired()) return tasks;

    // First phase: inspect every static goal cell and create direct delivery
    // tasks for goals that are not currently satisfied.
    for (int r = 0; r < level.rows; ++r) {
        if (deadline.expired()) return tasks;
        for (int c = 0; c < level.cols; ++c) {
            if (deadline.expired()) return tasks;
            const char goal = level.goal_at(r, c);

            // Non-box goals do not require moving a box, and already-satisfied
            // goals should not create redundant work.
            if (!is_box_goal(goal)) continue;
            if (state.box_at(r, c) == goal) continue;

            // Locate a currently unassigned box matching this goal symbol.
            // Prefer off-goal boxes closest to this goal so same-letter Sokoban
            // rows do not all target the first matching box found in row-major
            // order, and avoid stealing boxes that already satisfy another goal.
            Position box_pos = choose_box_for_goal(level, state, goal, Position{r, c}, assigned_boxes);

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
            assigned_boxes.push_back(box_pos);
            if (options.max_direct_delivery_tasks > 0 && tasks.size() >= options.max_direct_delivery_tasks) break;
        }
        if (options.max_direct_delivery_tasks > 0 && tasks.size() >= options.max_direct_delivery_tasks) break;
    }

    if (!options.include_blocker_tasks) {
        if (options.include_agent_goal_tasks && !deadline.expired()) append_agent_goal_tasks(level, state, next_task_id, tasks);
        return tasks;
    }

    // Second phase: augment direct deliveries with tasks that clear obstacles
    // detected by the level analysis. These blocker tasks share the same output
    // vector so solvers can plan deliveries and prerequisite clearing work from
    // one ordered task list.
    if (deadline.expired()) return tasks;
    LevelAnalyzer analyzer;
    const std::size_t signature = level_signature(level);
    if (!cached_initial_analysis_.has_value() || cached_level_signature_ != signature) {
        if (deadline.expired()) return tasks;
        cached_initial_analysis_ = analyzer.analyze(level, state, initial_agent_plans, deadline);
        cached_level_signature_ = signature;
    }
    if (deadline.expired()) return tasks;
    const LevelAnalysis analysis = analyzer.update(level, state, *cached_initial_analysis_, initial_agent_plans, deadline);
    if (deadline.expired()) return tasks;
    BlockerResolver blocker_resolver;
    // std::cerr << "Parking cells on the level" << static_cast<int>(analysis.parking_cells.size()) << std::endl;
    // for (int i = 0; i < static_cast<int>(analysis.parking_cells.size()); i++) std::cerr << "parking cell: " 
    // << analysis.parking_cells[i].to_string() << std::endl;
    std::vector<Task> blocker_tasks = blocker_resolver.generate_blocker_tasks(level, state, analysis, next_task_id, deadline);

    tasks.insert(tasks.end(), blocker_tasks.begin(), blocker_tasks.end());

    // Final phase: once all generated box/blocker work has completed, agents
    // that have digit goals should return to their own goal cells.  These tasks
    // are intentionally appended last and depend on the earlier task ids so they
    // do not steal an agent away before its useful work is done.
    if (options.include_agent_goal_tasks && !deadline.expired()) append_agent_goal_tasks(level, state, next_task_id, tasks);

    return tasks;
}
