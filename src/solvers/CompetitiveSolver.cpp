#include "solvers/CompetitiveSolver.hpp"

#include "actions/ActionSemantics.hpp"
#include "hospital/LocalRepair.hpp"
#include "plan/AgentPlan.hpp"
#include "plan/PlanConflictRepairer.hpp"
#include "plan/PlanMerger.hpp"
#include "solvers/CBSSolver.hpp"
#include "tasks/HTNTracePrinter.hpp"
#include "tasks/TaskGenerator.hpp"
#include "tasks/TaskPrioritizer.hpp"
#include "tasks/TaskScheduler.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <unordered_set>

namespace {
// Apply the first `horizon` joint actions of `plan` to a mutable State.
//
// Input:
//   - `state`: the simulated world before the prefix starts. It is updated in
//     place so the next competitive wave sees the boxes and agents where the
//     previous wave actually left them.
//   - `plan`: a sequence of synchronized actions, one JointAction per time step.
//   - `horizon`: the number of leading time steps that are considered safe to
//     commit to the final answer.
// Output:
//   - no return value; `state.agent_positions` and `state.boxes` are advanced as
//     if those actions had been executed by the competition server.
void apply_prefix(State& state, const Plan& plan, int horizon) {
    const int n = std::min(horizon, static_cast<int>(plan.steps.size()));
    for (int t = 0; t < n; ++t) {
        const JointAction& ja = plan.steps[t];
        for (int a = 0; a < state.num_agents() && a < static_cast<int>(ja.actions.size()); ++a) {
            const ActionEffect eff = ActionSemantics::compute_effect(state.agent_positions[a], ja.actions[a]);
            state.agent_positions[a] = eff.agent_to;
            if (eff.moves_box && state.in_bounds(eff.box_from.row, eff.box_from.col) && state.in_bounds(eff.box_to.row, eff.box_to.col)) {
                const char moved = state.box_at(eff.box_from.row, eff.box_from.col);
                if (moved != '\0') {
                    state.set_box(eff.box_from.row, eff.box_from.col, '\0');
                    state.set_box(eff.box_to.row, eff.box_to.col, moved);
                }
            }
        }
    }
}
// Read the optional environment toggle used while diagnosing HTN task choices.
// Verbose tracing is opt-in because benchmark stderr can become enormous and
// the competition server timeout includes that I/O overhead.
bool configured_verbose_tasks() {
    if (const char* v = std::getenv("MAPF_VERBOSE_TASKS")) {
        return std::atoi(v) != 0;
    }
    return false;
}

std::vector<std::string> box_layout_signature(const State& state) {
    std::vector<std::string> boxes;
    boxes.reserve(static_cast<std::size_t>(state.rows * state.cols));
    for (int r = 0; r < state.rows; ++r) {
        for (int c = 0; c < state.cols; ++c) {
            const char box = state.box_at(r, c);
            if (box == '\0') continue;
            boxes.push_back(std::string(1, box) + "@" + std::to_string(r) + "," + std::to_string(c));
        }
    }
    return boxes;
}

std::string world_signature(const State& state) {
    std::string signature;
    signature.reserve(static_cast<std::size_t>(state.rows * state.cols + state.num_agents() * 8));
    for (const Position& agent : state.agent_positions) {
        signature += "a" + std::to_string(agent.row) + "," + std::to_string(agent.col) + ";";
    }
    signature += "|";
    for (const std::string& box : box_layout_signature(state)) {
        signature += box + ";";
    }
    return signature;
}
}

CompetitiveSolver::CompetitiveSolver(SolverConfig config) : config_(config) {}

Plan CompetitiveSolver::solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) {
    // The competitive solver constructs plans at a higher level than A*: it
    // repeatedly decomposes the current state into delivery tasks, schedules a
    // short wave of box/agent movements, commits that wave, and then replans
    // from the new state. The generic heuristic is therefore unused here; the
    // task prioritizer and scheduler provide the domain-specific guidance.
    (void)heuristic;

    bool has_box_goal = false;
    for (int r = 0; r < level.rows && !has_box_goal; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const char goal = level.goal_at(r, c);
            if (goal >= 'A' && goal <= 'Z') {
                has_box_goal = true;
                break;
            }
        }
    }
    if (!has_box_goal) {
        CBSSolver fallback;
        return fallback.solve(level, initial_state, heuristic);
    }

    // Stop after several waves that fail to improve the state. This prevents an
    // infinite loop on unschedulable or cyclic task sets while still allowing a
    // few local-repair attempts to unblock progress.
    constexpr int kNoProgressBudget = 5;
    // Convert the external millisecond budget to the seconds representation used
    // by chrono::duration<double>. The main loop checks this before every wave.
    const double kTimeBudgetSeconds = static_cast<double>(config_.planning_time_budget_ms) / 1000.0;

    // Verbose mode is intentionally resolved once so a single solve call has a
    // consistent trace format even if the environment changes mid-run.
    const bool kVerboseTasks = config_.debug_htn_trace || configured_verbose_tasks();
    const auto start_time = std::chrono::steady_clock::now();

    // `current` is the solver's internal simulation of the world after all
    // committed waves. `accumulated` is the output plan returned to the client.
    State current = initial_state;
    Plan accumulated;

    // Pipeline components for one competitive wave:
    //   1. generate high-level delivery tasks (with cached level analysis),
    //   2. score them for traceability,
    //   3. schedule primitive actions with reservations, and
    //   4. try targeted repair if scheduling fails.
    TaskGenerator generator;
    TaskScheduler scheduler;
    TaskPrioritizer prioritizer;
    LocalRepair repair;
    PlanConflictRepairer conflict_repairer;
    int no_progress_iters = 0;
    int wave_count = 0;
    const int max_waves = std::max(32, level.rows * level.cols * 4);
    std::unordered_set<std::string> seen_worlds;
    seen_worlds.insert(world_signature(current));

    // Count completed box goals in a State. The value is used only as a simple
    // progress signal between waves; task generation still handles the detailed
    // compatibility checks for agents, boxes, and colors.
    const auto goals_completed = [&level](const State& s) {
        int done = 0;
        for (int r = 0; r < level.rows; ++r) for (int c = 0; c < level.cols; ++c) {
            const char g = level.goal_at(r, c);
            if (g >= 'A' && g <= 'Z' && s.box_at(r, c) == g) ++done;
        }
        return done;
    };
    int best_goals_completed = goals_completed(current);

    // Main HTN loop: each iteration plans one wave from the latest simulated
    // state. Inputs are the level and `current`; output is appended into
    // `accumulated` and reflected back into `current` via apply_prefix().
    while (true) {
        if (++wave_count > max_waves) break;
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - start_time).count();
        if (elapsed >= kTimeBudgetSeconds) break;

        // Convert unsatisfied goals into high-level delivery tasks. Each Task is
        // an intuitive contract: move a compatible box/agent from its source to
        // its target goal while respecting hospital-domain constraints.
        std::vector<Task> tasks = generator.generate_delivery_tasks(level, current);
        if (kVerboseTasks) {
            // Scoring is done on a copy because trace output should explain the
            // scheduler's priorities without mutating the task list returned by
            // the generator.
            std::vector<Task> scored = tasks;
            prioritizer.score(level, current, scored);
            std::cerr << "[HTN] competitive_wave tasks=" << scored.size()
                      << " elapsed=" << elapsed << "s\n";
            HTNTracePrinter::print_task_batch(scored, std::cerr);
            for (const auto& reason : generator.skip_reasons()) {
                std::cerr << "[HTN] " << reason << '\n';
            }
        }
        // No tasks means every currently generatable delivery goal is already
        // satisfied or impossible to assign, so there is no useful competitive
        // wave to append.
        if (tasks.empty()) break;

        // Ask the scheduler to turn high-level tasks into a concrete joint plan.
        // Input: current world state plus the candidate tasks. Output: a wave of
        // synchronized actions that can be appended to the final plan.
        std::vector<AgentPlan> wave_agent_plans = scheduler.build_agent_plans(level, current, tasks);
        Plan wave = PlanMerger::merge_agent_plans(wave_agent_plans, current.num_agents());
        PlanMerger::compact_independent_actions(level, current, wave);
        if (wave.empty()) {
            // An empty wave means the scheduler could not safely reserve paths
            // for the generated tasks. Local repair tries one focused fallback
            // around the first task before the main loop gives up.
            if (kVerboseTasks) {
                std::cerr << "[HTN] scheduler_empty; attempting local repair on first task\n";
            }
            TaskPlan failed;
            failed.success = false;
            failed.failure_reason = "scheduler_empty";
            for (int i = 0 ; i < static_cast<int>(tasks.size()); i++){
                const RepairResult repaired = repair.repair(level, current, tasks[i], failed);
                if (kVerboseTasks) {
                    std::cerr << "[HTN] repair outcome=" << static_cast<int>(repaired.outcome)
                            << " success=" << (repaired.plan.success ? "true" : "false") << '\n';
                }
                if (repaired.plan.success && repaired.outcome == RepairStageOutcome::SafePrefixFallback && !accumulated.steps.empty()) {
                    // Repair may validate that the already accumulated safe prefix is
                    // the best available output. Reset the stagnation counter so the
                    // solver can re-analyze from the unchanged state once more.
                    no_progress_iters = 0;
                    continue;
                }
                if (++no_progress_iters >= kNoProgressBudget) break;
                continue;
            }
        }
        
        // The scheduler normally uses reservations, but a competitive wave can
        // still expose a conflict once the selected task/agent timelines are
        // composed. Repair operates on the AgentPlan wave directly so it can add
        // conflict constraints and replan alternate routes without changing task
        // priorities or reshuffling the accepted schedule.
        // for (JointAction ja : wave.steps) std::cerr << ja.to_string() << std::endl;
        const PlanConflictRepairer::Result repaired_wave = conflict_repairer.repair(level, current, wave_agent_plans);
        if (repaired_wave.conflict_free) {
            if (kVerboseTasks && repaired_wave.changed) {
                std::cerr << "[HTN] cbs_style_repair inserted waits iterations="
                          << repaired_wave.iterations << " steps=" << repaired_wave.plan.steps.size() << '\n';
            }
            wave_agent_plans = repaired_wave.agent_plans;
            wave = PlanMerger::merge_agent_plans(wave_agent_plans, current.num_agents());
            PlanMerger::compact_independent_actions(level, current, wave);
        } else if (kVerboseTasks) {
            std::cerr << "[HTN] cbs_style_repair could not prove conflict-free wave; using scheduler wave\n";
        }

        // Snapshot state before committing the wave so we can determine whether
        // it made meaningful progress after simulation.
        const State before = current;
        const int goals_before = goals_completed(before);
        const std::vector<std::string> boxes_before = box_layout_signature(before);

        // Today the scheduler returns only executable safe waves, so the safe
        // prefix is the complete wave. Keeping the prefix variable makes the
        // input/output boundary explicit and leaves room for future partial-wave
        // conflict trimming.
        const int safe_prefix = static_cast<int>(wave.steps.size());
        for (int i = 0; i < safe_prefix; ++i) {
            accumulated.steps.push_back(wave.steps[i]);
        }
        // Append the selected prefix to the final output, then mirror the same
        // actions into `current` so the next iteration plans from the expected
        // post-wave state.
        apply_prefix(current, wave, safe_prefix);
        const int goals_after = goals_completed(current);
        const bool repeated_world = !seen_worlds.insert(world_signature(current)).second;
        const bool completed_new_goal = goals_after > goals_before;
        const bool moved_any_box = box_layout_signature(current) != boxes_before;
        if (repeated_world) {
            break;
        }
        if (completed_new_goal || (moved_any_box && goals_after >= best_goals_completed)) {
            best_goals_completed = std::max(best_goals_completed, goals_after);
            no_progress_iters = 0;
        } else if (++no_progress_iters >= kNoProgressBudget) {
            break;
        }
    }

    // Return the best HTN/reservation prefix immediately.  The old CBS fallback
    // had no wall-clock budget and could consume the remaining server time after
    // the competitive solver had already found useful work, turning partial
    // failures into hard timeouts.
    return accumulated;
}
