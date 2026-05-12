#include "tasks/TaskScheduler.hpp"

#include "actions/ActionSemantics.hpp"
#include "analysis/LevelAnalyzer.hpp"
#include "analysis/ParkingCellAnalyzer.hpp"
#include "hospital/AgentPathPlanner.hpp"
#include "hospital/BoxTransportPlanner.hpp"
#include "plan/ConflictDetector.hpp"
#include "plan/PlanMerger.hpp"
#include "plan/ReservationTable.hpp"
#include "tasks/DependencyBuilder.hpp"
#include "tasks/TaskPrioritizer.hpp"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>

namespace {
// Simple distance heuristic used only for ranking candidate agents.  It ignores
// walls, boxes, and reservations because the real planners below validate the
// path; the scheduler only needs a cheap "try nearby agents first" ordering.
int manhattan(const Position& a, const Position& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

bool verbose_scheduler() {
    if (const char* v = std::getenv("MAPF_VERBOSE_TASKS")) {
        return std::atoi(v) != 0;
    }
    return false;
}

bool is_satisfied_box_goal(const Level& level, const State& state, Position pos, char box_id) {
    return level.goal_at(pos.row, pos.col) == box_id && state.box_at(pos.row, pos.col) == box_id;
}

Position choose_current_box_position(const Level& level, const State& state, const Task& task) {
    if (state.in_bounds(task.box_pos.row, task.box_pos.col) &&
        state.box_at(task.box_pos.row, task.box_pos.col) == task.box_id) {
        return task.box_pos;
    }

    Position best{-1, -1};
    int best_score = std::numeric_limits<int>::max();
    for (int r = 0; r < state.rows; ++r) {
        for (int c = 0; c < state.cols; ++c) {
            if (state.box_at(r, c) != task.box_id) continue;
            const Position candidate{r, c};
            const int satisfied_goal_penalty = is_satisfied_box_goal(level, state, candidate, task.box_id) ? 100000 : 0;
            const int score = satisfied_goal_penalty + manhattan(candidate, task.goal_pos);
            if (score < best_score) {
                best_score = score;
                best = candidate;
            }
        }
    }
    return best;
}

// Return the agents that are allowed to execute a task, ordered by preference.
//
// Input:
//   - level: supplies static agent/box color ownership rules.
//   - state: supplies current simulated positions for distance scoring.
//   - task: the high-level work item that needs an executor.
// Output:
//   - agent ids that may legally do the task.  Agent-only tasks keep their
//     requested agent, while box tasks return all same-colored agents sorted by
//     their Manhattan distance to the current box position.

std::vector<Position> parking_targets_for_task(const Task& task) {
    std::vector<Position> targets;
    if (task.type != TaskType::MoveBlockingBoxToParking) return targets;

    auto add = [&](Position p) {
        if (p.row < 0 || p.col < 0) return;
        if (std::find(targets.begin(), targets.end(), p) == targets.end()) targets.push_back(p);
    };
    add(task.parking_pos);
    for (Position p : task.parking_candidates) add(p);
    return targets;
}

std::vector<int> candidate_agents_for_task(const Level& level, const State& state, const Task& task) {
    std::vector<std::pair<int, int>> scored;

    // Agent-only tasks are already bound to one agent by the task generator.
    // Returning any other agent would change the meaning of the task.
    if (task.type == TaskType::MoveAgentToGoal || task.type == TaskType::ParkAgentSafely) {
        if (task.agent_id >= 0 && task.agent_id < state.num_agents()) return {task.agent_id};
        return {};
    }

    // Box-moving tasks must be executed by an agent with the same color as the
    // box.  Invalid or missing box identifiers make the task unschedulable.
    if (task.box_id < 'A' || task.box_id > 'Z') return {};
    const Color box_color = level.box_colors[static_cast<std::size_t>(task.box_id - 'A')];
    for (int a = 0; a < state.num_agents(); ++a) {
        if (level.agent_colors[static_cast<std::size_t>(a)] != box_color) continue;
        scored.emplace_back(manhattan(state.agent_positions[static_cast<std::size_t>(a)], task.box_pos), a);
    }

    std::sort(scored.begin(), scored.end());
    std::vector<int> out;
    for (const auto& [_, a] : scored) out.push_back(a);
    return out;
}

// Some tasks are already satisfied when the scheduler reaches them.  An empty
// primitive action list is only acceptable in those cases; otherwise accepting it
// would mark unfinished work as completed.
bool is_empty_plan_valid(const Task& task, const State& state) {
    if (task.type == TaskType::DeliverBoxToGoal) return task.box_pos == task.goal_pos;
    if (task.type == TaskType::MoveAgentToGoal || task.type == TaskType::ParkAgentSafely) {
        return task.agent_id >= 0 && task.agent_id < state.num_agents() && state.agent_positions[task.agent_id] == task.goal_pos;
    }
    return true;
}

int agent_at_position(const State& state, Position pos, int ignored_agent = -1) {
    for (int agent = 0; agent < state.num_agents(); ++agent) {
        if (agent == ignored_agent) continue;
        if (state.agent_positions[static_cast<std::size_t>(agent)] == pos) return agent;
    }
    return -1;
}

bool is_free_non_goal_agent_parking_cell(const Level& level, const State& state, Position pos, int moving_agent) {
    if (!level.in_bounds(pos.row, pos.col) || level.is_wall(pos.row, pos.col)) return false;
    if (level.goal_at(pos.row, pos.col) != '\0') return false;
    if (state.has_box(pos.row, pos.col)) return false;
    return agent_at_position(state, pos, moving_agent) == -1;
}

std::vector<Position> parking_candidates_for_agent(const Level& level, const State& state, int moving_agent, const PlanningDeadline& deadline) {
    if (deadline.expired()) return {};
    LevelAnalyzer analyzer;
    const LevelAnalysis analysis = analyzer.analyze(level, state);
    if (deadline.expired()) return {};
    ParkingCellAnalyzer parking_analyzer;
    std::vector<Position> candidates = parking_analyzer.find_parking_cells(level, state, analysis);

    for (const Position pos : analysis.free_cells) {
        if (deadline.expired()) break;
        if (std::find(candidates.begin(), candidates.end(), pos) != candidates.end()) continue;
        candidates.push_back(pos);
    }

    const Position start = state.agent_positions[static_cast<std::size_t>(moving_agent)];
    candidates.erase(std::remove_if(candidates.begin(), candidates.end(), [&](Position pos) {
        return pos == start || !is_free_non_goal_agent_parking_cell(level, state, pos, moving_agent);
    }), candidates.end());

    std::stable_sort(candidates.begin(), candidates.end(), [&](Position a, Position b) {
        const int score_a = analysis.at(a).parking_score;
        const int score_b = analysis.at(b).parking_score;
        if (score_a != score_b) return score_a > score_b;
        return manhattan(start, a) < manhattan(start, b);
    });
    return candidates;
}

TaskPlan plan_agent_parking(const Level& level,
                            const State& state,
                            int moving_agent,
                            const ReservationTable& reservations,
                            int start_time,
                            int task_id,
                            Position blocked_goal,
                            const PlanningDeadline& deadline) {
    AgentPathPlanner agent_planner;
    TaskPlan best_failure;
    best_failure.task_id = task_id;
    best_failure.task_type = TaskType::ParkAgentSafely;
    best_failure.agent_id = moving_agent;
    best_failure.failure_reason = "no_safe_agent_parking_cell";

    for (const Position candidate : parking_candidates_for_agent(level, state, moving_agent, deadline)) {
        if (deadline.expired()) break;
        Task park;
        park.type = TaskType::ParkAgentSafely;
        park.task_id = task_id;
        park.agent_id = moving_agent;
        park.goal_pos = candidate;
        park.parking_pos = candidate;
        park.box_pos = state.agent_positions[static_cast<std::size_t>(moving_agent)];
        park.priority = std::numeric_limits<int>::max() / 4;
        park.debug_label = "clear_agent_from_goal_(" + std::to_string(blocked_goal.row) + "," + std::to_string(blocked_goal.col) + ")";

        TaskPlan plan = agent_planner.plan(level, state, park, reservations, start_time, deadline);
        if (plan.success && !plan.agent_plan.actions.empty()) return plan;
        if (!plan.failure_reason.empty()) best_failure.failure_reason = plan.failure_reason;
    }

    return best_failure;
}

void replay_task_plan_into_state(State& simulated_state, const Task& task, const TaskPlan& plan) {
    Position cur = simulated_state.agent_positions[static_cast<std::size_t>(task.agent_id)];
    for (const Action& a : plan.agent_plan.actions) {
        const ActionEffect eff = ActionSemantics::compute_effect(cur, a);
        cur = eff.agent_to;
        if (eff.moves_box && simulated_state.in_bounds(eff.box_from.row, eff.box_from.col) && simulated_state.in_bounds(eff.box_to.row, eff.box_to.col)) {
            const char moved = simulated_state.box_at(eff.box_from.row, eff.box_from.col);
            if (moved != '\0') {
                simulated_state.set_box(eff.box_from.row, eff.box_from.col, '\0');
                simulated_state.set_box(eff.box_to.row, eff.box_to.col, moved);
            }
        }
    }
    simulated_state.agent_positions[static_cast<std::size_t>(task.agent_id)] = cur;
}

std::vector<AgentPlan> expand_scheduled_tasks(const State& initial_state, const std::vector<ScheduledTask>& scheduled) {
    std::vector<AgentPlan> agent_plans(initial_state.num_agents());
    if (scheduled.empty()) return {};
    for (int agent = 0; agent < initial_state.num_agents(); ++agent) {
        agent_plans[agent].agent = agent;
    }
    for (const auto& st : scheduled) {
        auto& timeline = agent_plans[st.task.agent_id].actions;
        if (static_cast<int>(timeline.size()) < st.start_time) timeline.resize(st.start_time, Action::noop());
        if (static_cast<int>(timeline.size()) < st.end_time) timeline.resize(st.end_time, Action::noop());
        for (int i = 0; i < static_cast<int>(st.plan.agent_plan.actions.size()); ++i) {
            timeline[st.start_time + i] = st.plan.agent_plan.actions[static_cast<std::size_t>(i)];
        }
    }

    // Reconstruct positions from actions for each agent because PlanMerger and
    // downstream validators consume both the action sequence and the derived
    // trajectory.  The input is an action timeline; the output is positions[0]
    // at the initial state plus one position after each action.
    for (int agent = 0; agent < initial_state.num_agents(); ++agent) {
        AgentPlan& plan = agent_plans[agent];
        Position cur = initial_state.agent_positions[agent];
        plan.positions.clear();
        plan.positions.push_back(cur);
        for (const Action& action : plan.actions) {
            cur = ActionSemantics::compute_effect(cur, action).agent_to;
            plan.positions.push_back(cur);
        }
    }

    return agent_plans;
}

Plan make_uncompressed_plan(const std::vector<AgentPlan>& agent_plans, int num_agents) {
    std::size_t horizon = 0;
    for (const AgentPlan& plan : agent_plans) horizon = std::max(horizon, plan.actions.size());

    Plan out;
    out.steps.reserve(horizon);
    for (std::size_t t = 0; t < horizon; ++t) {
        JointAction step;
        step.actions.resize(num_agents, Action::noop());
        for (int agent = 0; agent < num_agents; ++agent) {
            if (t < agent_plans[agent].actions.size()) {
                step.actions[agent] = agent_plans[agent].actions[t];
            }
        }
        out.steps.push_back(std::move(step));
    }
    return out;
}

const ScheduledTask* task_active_for_agent(const std::vector<ScheduledTask>& scheduled, int agent_id, int time) {
    if (agent_id < 0) return nullptr;
    for (const ScheduledTask& st : scheduled) {
        if (st.task.agent_id != agent_id) continue;
        if (st.start_time <= time && time < st.end_time) return &st;
    }
    if (time > 0) {
        for (const ScheduledTask& st : scheduled) {
            if (st.task.agent_id != agent_id) continue;
            if (st.start_time <= time - 1 && time - 1 < st.end_time) return &st;
        }
    }
    return nullptr;
}

const ScheduledTask* task_active_for_box(const std::vector<ScheduledTask>& scheduled, char box_letter, int time) {
    if (box_letter == '\0') return nullptr;
    for (const ScheduledTask& st : scheduled) {
        if (st.task.box_id != box_letter) continue;
        if (st.start_time <= time && time < st.end_time) return &st;
    }
    if (time > 0) {
        for (const ScheduledTask& st : scheduled) {
            if (st.task.box_id != box_letter) continue;
            if (st.start_time <= time - 1 && time - 1 < st.end_time) return &st;
        }
    }
    return nullptr;
}

void add_unique_task(std::vector<const ScheduledTask*>& tasks, const ScheduledTask* task) {
    if (task == nullptr) return;
    const auto same_id = [task](const ScheduledTask* other) {
        return other != nullptr && other->task.task_id == task->task.task_id;
    };
    if (std::find_if(tasks.begin(), tasks.end(), same_id) == tasks.end()) tasks.push_back(task);
}

std::optional<std::pair<int, int>> dependency_from_conflict(const std::vector<ScheduledTask>& scheduled, const Conflict& conflict) {
    std::vector<const ScheduledTask*> involved;
    add_unique_task(involved, task_active_for_agent(scheduled, conflict.agents[0], conflict.time));
    add_unique_task(involved, task_active_for_agent(scheduled, conflict.agents[1], conflict.time));
    add_unique_task(involved, task_active_for_box(scheduled, conflict.box_letters[0], conflict.time));
    add_unique_task(involved, task_active_for_box(scheduled, conflict.box_letters[1], conflict.time));
    if (involved.size() < 2) return std::nullopt;

    std::sort(involved.begin(), involved.end(), [](const ScheduledTask* a, const ScheduledTask* b) {
        if (a->task.priority != b->task.priority) return a->task.priority > b->task.priority;
        return a->task.task_id < b->task.task_id;
    });

    // The task that should run first becomes the predecessor.  Prefer the task
    // that the prioritizer ranked higher, and use task id as a stable tie-break.
    return std::make_pair(involved.front()->task.task_id, involved.back()->task.task_id);
}

bool add_dependency(std::vector<Task>& tasks, int predecessor, int successor) {
    for (Task& task : tasks) {
        if (task.task_id != successor) continue;
        if (std::find(task.dependencies.begin(), task.dependencies.end(), predecessor) != task.dependencies.end()) return false;
        task.dependencies.push_back(predecessor);
        return true;
    }
    return false;
}

bool is_box_goal(char goal) {
    return goal >= 'A' && goal <= 'Z';
}

void add_corridor_access_dependencies(const Level& level, const State& state, std::vector<Task>& tasks) {
    for (const Task& far_task : tasks) {
        if (far_task.type != TaskType::DeliverBoxToGoal) continue;
        if (far_task.agent_id < 0 || far_task.agent_id >= state.num_agents()) continue;

        const Position agent = state.agent_positions[static_cast<std::size_t>(far_task.agent_id)];
        if (agent.row != far_task.goal_pos.row || agent.col == far_task.goal_pos.col) continue;

        const int step = (far_task.goal_pos.col > agent.col) ? 1 : -1;
        for (int c = agent.col + step; c != far_task.goal_pos.col; c += step) {
            if (!level.in_bounds(agent.row, c) || level.is_wall(agent.row, c)) break;
            const char goal = level.goal_at(agent.row, c);
            if (!is_box_goal(goal) || goal == far_task.box_id) continue;
            if (state.box_at(agent.row, c) == goal) continue;

            for (Task& near_task : tasks) {
                if (near_task.type != TaskType::DeliverBoxToGoal) continue;
                if (near_task.task_id == far_task.task_id) continue;
                if (near_task.goal_pos == Position{agent.row, c}) {
                    add_dependency(tasks, far_task.task_id, near_task.task_id);
                    break;
                }
            }
        }
    }
}

std::vector<ScheduledTask> schedule_once(
    const Level& level,
    const State& initial_state,
    const std::vector<Task>& mutable_tasks,
    const PlanningDeadline& deadline
) {
    // Global reservation state is the contract between independently planned
    // tasks.  Each successful task contributes occupied cells/edges to this
    // table so later planning attempts avoid collisions in space and time.
    ReservationTable reservations;
    if (deadline.expired()) return {};
    std::vector<ScheduledTask> scheduled;

    // The scheduler delegates low-level path construction to specialized
    // planners, and only decides which task/agent/start-time combination to try.
    BoxTransportPlanner box_planner;
    AgentPathPlanner agent_planner;
    DependencyBuilder deps_builder;

    // simulated_state is advanced after every accepted task.  It represents the
    // world at the frontier of the partial schedule, so later tasks see the new
    // agent positions and current box locations rather than the original input.
    State simulated_state = initial_state;
    const auto deps = deps_builder.build_graph(mutable_tasks);

    // completed/completion_time track the dependency graph.  agent_available is
    // the earliest timestep at which each agent can accept another task.
    std::unordered_set<int> completed;
    std::unordered_map<int, int> completion_time;
    std::vector<int> agent_available(initial_state.num_agents(), 0);
    int next_dynamic_task_id = -1;

    // Repeatedly schedule every currently ready task that can be planned.  The
    // loop stops when a full pass cannot add anything, which means all remaining
    // tasks are either blocked by dependencies or currently unplannable.
    bool progress = true;
    while (progress) {
        if (deadline.expired()) break;
        progress = false;

        // Convert ready task ids back into Task objects, then process higher
        // priority work first.  Dependencies define what is legal; priority
        // defines what is preferred among the legal choices.
        std::vector<Task> ready;
        const auto ready_ids = deps.ready_tasks(completed);
        for (const Task& t : mutable_tasks) {
            if (std::find(ready_ids.begin(), ready_ids.end(), t.task_id) != ready_ids.end()) ready.push_back(t);
        }
        std::sort(ready.begin(), ready.end(), [](const Task& a, const Task& b) { return a.priority > b.priority; });

        // Within one scheduling pass, give each agent at most one newly accepted
        // task.  This prevents a single agent from consuming the entire ready
        // queue before other agents get a chance to plan work in parallel.
        std::unordered_set<int> used_agents;

        for (const Task& task : ready) {
            if (deadline.expired()) return scheduled;
            // A task cannot start before all of its predecessors have ended.
            // Missing predecessors are not ready, so completion_time[pre] is
            // expected to exist for every predecessor seen here.
            int dep_time = 0;
            auto it = deps.predecessors.find(task.task_id);
            if (it != deps.predecessors.end()) {
                for (int pre : it->second) dep_time = std::max(dep_time, completion_time[pre]);
            }

            // Refresh the box position from the simulated world.  Earlier tasks
            // may have moved the same box after task creation, so the planner
            // needs the current input position, not stale task metadata.
            Task chosen_task = task;
            if (chosen_task.box_id >= 'A' && chosen_task.box_id <= 'Z') {
                chosen_task.box_pos = choose_current_box_position(level, simulated_state, chosen_task);
            }

            // If another agent is currently parked on this delivery's goal,
            // synthesize and schedule a just-in-time ParkAgentSafely task for
            // that blocker before attempting the box planner.  This turns the
            // detected dynamic obstruction into explicit work instead of letting
            // the following delivery fail against a static occupied goal cell.
            if (chosen_task.type == TaskType::DeliverBoxToGoal) {
                const int blocking_agent = agent_at_position(simulated_state, chosen_task.goal_pos, chosen_task.agent_id);
                if (blocking_agent >= 0) {
                    const int parking_start = std::max(agent_available[static_cast<std::size_t>(blocking_agent)], dep_time);
                    const int parking_task_id = next_dynamic_task_id--;
                    TaskPlan parking_plan = plan_agent_parking(level,
                                                               simulated_state,
                                                               blocking_agent,
                                                               reservations,
                                                               parking_start,
                                                               parking_task_id,
                                                               chosen_task.goal_pos,
                                                               deadline);
                    if (parking_plan.success) {
                        Task parking_task;
                        parking_task.type = TaskType::ParkAgentSafely;
                        parking_task.task_id = parking_task_id;
                        parking_task.agent_id = blocking_agent;
                        parking_task.goal_pos = parking_plan.agent_plan.positions.back();
                        parking_task.parking_pos = parking_task.goal_pos;
                        parking_task.box_pos = simulated_state.agent_positions[static_cast<std::size_t>(blocking_agent)];
                        parking_task.priority = std::numeric_limits<int>::max() / 4;
                        parking_task.unblocks_box_id = chosen_task.box_id;
                        parking_task.debug_label = "dynamic_goal_agent_blocker_for_" + std::string(1, chosen_task.box_id);

                        const int parking_end = parking_start + static_cast<int>(parking_plan.agent_plan.actions.size());
                        scheduled.push_back(ScheduledTask{parking_task, parking_plan, parking_start, parking_end});
                        reservations.reserve_path(parking_plan.agent_plan, parking_start);
                        replay_task_plan_into_state(simulated_state, parking_task, parking_plan);
                        agent_available[static_cast<std::size_t>(blocking_agent)] = parking_end;
                        dep_time = std::max(dep_time, parking_end);
                        progress = true;
                        if (verbose_scheduler()) {
                            std::cerr << "[HTN] dynamic_agent_goal_blocker agent=" << blocking_agent
                                      << " goal=(" << chosen_task.goal_pos.row << "," << chosen_task.goal_pos.col << ")"
                                      << " park=(" << parking_task.parking_pos.row << "," << parking_task.parking_pos.col << ")"
                                      << " unblock=" << chosen_task.box_id << '\n';
                        }
                    } else {
                        if (verbose_scheduler()) {
                            std::cerr << "[HTN] dynamic_agent_goal_blocker_failed agent=" << blocking_agent
                                      << " goal=(" << chosen_task.goal_pos.row << "," << chosen_task.goal_pos.col << ")"
                                      << " reason=" << parking_plan.failure_reason << '\n';
                        }
                    }
                }
            }

            // Try legal candidate agents in preference order.  The first
            // successful low-level plan becomes the schedule entry for this
            // high-level task; failed candidates leave the task available for a
            // later pass after other reservations/state may have changed.
            TaskPlan plan;
            int chosen_start = 0;
            bool planned = false;
            const std::vector<int> candidates = candidate_agents_for_task(level, simulated_state, chosen_task);
            for (int candidate_agent : candidates) {
                if (candidate_agent < 0 || candidate_agent >= initial_state.num_agents()) continue;
                if (used_agents.count(candidate_agent)) continue;

                chosen_task.agent_id = candidate_agent;
                chosen_start = std::max(agent_available[candidate_agent], dep_time);

                // Agent-only tasks use point-to-point path planning.  Box tasks
                // use transport planning, which plans both agent actions and the
                // box trajectory starting at chosen_start in the global timeline.
                if (chosen_task.type == TaskType::MoveAgentToGoal || chosen_task.type == TaskType::ParkAgentSafely) {
                    if (verbose_scheduler()) {
                        std::cerr << "path planning for task: " << chosen_task.task_id << " and assigned agent: " << chosen_task.agent_id << std::endl;
                    }
                    plan = agent_planner.plan(level, simulated_state, chosen_task, reservations, chosen_start, deadline);
                    if (verbose_scheduler()) {
                        std::cerr << "success: " << (plan.success ? "true" : "false") << std::endl;
                    }
                } else if (chosen_task.type == TaskType::MoveBlockingBoxToParking) {
                    const std::vector<Position> parking_targets = parking_targets_for_task(chosen_task);
                    if (verbose_scheduler()) {
                        std::cerr << "box planning for task: " << chosen_task.task_id
                                  << " and assigned agent: " << chosen_task.agent_id
                                  << " parking_candidates=" << parking_targets.size() << std::endl;
                    }
                    for (const Position parking_target : parking_targets) {
                        chosen_task.parking_pos = parking_target;
                        chosen_task.goal_pos = parking_target;
                        plan = box_planner.plan(level, simulated_state, chosen_task, reservations, chosen_start, deadline);
                        if (verbose_scheduler()) {
                            std::cerr << "success: " << (plan.success ? "true" : "false")
                                      << " parking=(" << parking_target.row << "," << parking_target.col << ")"
                                      << " reason: " << plan.failure_reason << std::endl;
                        }
                        if (plan.success && (!plan.agent_plan.actions.empty() || is_empty_plan_valid(chosen_task, simulated_state))) break;
                    }
                } else {
                    if (verbose_scheduler()) {
                        std::cerr << "box planning for task: " << chosen_task.task_id << " and assigned agent: " << chosen_task.agent_id << std::endl;
                    }
                    plan = box_planner.plan(level, simulated_state, chosen_task, reservations, chosen_start, deadline);
                    if (verbose_scheduler()) {
                        std::cerr << "success: " << (plan.success ? "true" : "false") << " reason: " << plan.failure_reason << std::endl;
                    }
                }

                if (!plan.success) continue;
                if (plan.agent_plan.actions.empty() && !is_empty_plan_valid(chosen_task, simulated_state)) continue;
                planned = true;
                break;
            }
            if (!planned) continue;

            // Commit the accepted single-task plan to the global schedule.
            // Output of this section is a ScheduledTask plus reservation entries
            // that later tasks must respect.
            const int end_time = chosen_start + static_cast<int>(plan.agent_plan.actions.size());
            scheduled.push_back(ScheduledTask{chosen_task, plan, chosen_start, end_time});
            reservations.reserve_path(plan.agent_plan, chosen_start);
            if (chosen_task.box_id >= 'A' && chosen_task.box_id <= 'Z' && !plan.box_trajectory.empty()) {
                reservations.reserve_box_path(chosen_task.box_id, plan.box_trajectory, chosen_start);
            }

            // Replay the accepted primitive actions into simulated_state.  This
            // updates the scheduler's working copy of agent and box positions so
            // the next task receives intuitive inputs: the world after all work
            // accepted so far has completed.
            replay_task_plan_into_state(simulated_state, chosen_task, plan);

            // Mark the task as finished in both the dependency graph and the
            // per-agent timeline.  These values become inputs to future start
            // time calculations.
            completed.insert(chosen_task.task_id);
            completion_time[chosen_task.task_id] = end_time;
            agent_available[chosen_task.agent_id] = end_time;
            used_agents.insert(chosen_task.agent_id);
            progress = true;
        }
    }

    return scheduled;
}
}

std::vector<AgentPlan> TaskScheduler::build_agent_plans(const Level& level, const State& initial_state, const std::vector<Task>& tasks) const {
    PlanningDeadline no_deadline;
    return build_agent_plans(level, initial_state, tasks, no_deadline);
}

std::vector<AgentPlan> TaskScheduler::build_agent_plans(const Level& level, const State& initial_state, const std::vector<Task>& tasks, const PlanningDeadline& deadline) const {
    TaskPrioritizer prioritizer;
    std::vector<Task> mutable_tasks = tasks;
    prioritizer.score(level, initial_state, mutable_tasks);
    add_corridor_access_dependencies(level, initial_state, mutable_tasks);

    // Schedule first, then validate the composed result.  If independently
    // planned tasks still conflict, convert the conflict into an explicit
    // dependency and retry so the next pass plans the successor after the
    // predecessor's committed finish time.
    const int max_dependency_repairs = std::max(1, static_cast<int>(mutable_tasks.size()) * static_cast<int>(mutable_tasks.size()));
    std::vector<ScheduledTask> scheduled;
    for (int repair = 0; repair <= max_dependency_repairs; ++repair) {
        if (deadline.expired()) break;
        scheduled = schedule_once(level, initial_state, mutable_tasks, deadline);
        std::vector<AgentPlan> agent_plans = expand_scheduled_tasks(initial_state, scheduled);
        if (agent_plans.empty()) return {};

        const Plan raw_plan = make_uncompressed_plan(agent_plans, initial_state.num_agents());
        const Conflict conflict = ConflictDetector::findFirstConflict(raw_plan, initial_state);
        if (!conflict.valid()) return agent_plans;

        const auto dependency = dependency_from_conflict(scheduled, conflict);
        if (!dependency.has_value()) {
            if (verbose_scheduler()) {
                std::cerr << "[HTN] unresolved schedule conflict without task mapping: " << to_string(conflict) << '\n';
            }
            return agent_plans;
        }

        const auto [predecessor, successor] = *dependency;
        if (!add_dependency(mutable_tasks, predecessor, successor)) {
            if (verbose_scheduler()) {
                std::cerr << "[HTN] unresolved schedule conflict for existing dependency "
                          << predecessor << " -> " << successor << ": " << to_string(conflict) << '\n';
            }
            return agent_plans;
        }

        if (verbose_scheduler()) {
            std::cerr << "[HTN] conflict_dependency " << predecessor << " -> " << successor
                      << " due to " << to_string(conflict) << '\n';
        }
    }

    return expand_scheduled_tasks(initial_state, scheduled);
}

Plan TaskScheduler::build_plan(const Level& level, const State& initial_state, const std::vector<Task>& tasks) const {
    PlanningDeadline no_deadline;
    return build_plan(level, initial_state, tasks, no_deadline);
}

Plan TaskScheduler::build_plan(const Level& level, const State& initial_state, const std::vector<Task>& tasks, const PlanningDeadline& deadline) const {
    const std::vector<AgentPlan> agent_plans = build_agent_plans(level, initial_state, tasks, deadline);
    if (agent_plans.empty()) return {};
    Plan plan = PlanMerger::merge_agent_plans(agent_plans, initial_state.num_agents());
    PlanMerger::compact_independent_actions(level, initial_state, plan);
    return plan;
}
