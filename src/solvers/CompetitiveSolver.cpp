#include "solvers/CompetitiveSolver.hpp"

#include "actions/ActionSemantics.hpp"
#include "hospital/LocalRepair.hpp"
#include "plan/AgentPlan.hpp"
#include "plan/PlanConflictRepairer.hpp"
#include "plan/PlanMerger.hpp"
#include "plan/ReservationTable.hpp"
#include "plan/WaveProgressGuard.hpp"
#include "solvers/CBSSolver.hpp"
#include "tasks/HTNTracePrinter.hpp"
#include "tasks/TaskGenerator.hpp"
#include "tasks/TaskPrioritizer.hpp"
#include "tasks/TaskScheduler.hpp"
#include "utils/PlanningDeadline.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>

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

int total_box_goals(const Level& level) {
    int total = 0;
    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const char goal = level.goal_at(r, c);
            if (goal >= 'A' && goal <= 'Z') ++total;
        }
    }
    return total;
}

bool has_planned_actions(const std::vector<AgentPlan>& agent_plans) {
    return std::any_of(agent_plans.begin(),
                       agent_plans.end(),
                       [](const AgentPlan& plan) { return !plan.actions.empty(); });
}

std::string compact_stop_reason(std::string reason) {
    for (char& ch : reason) {
        const bool safe = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                          (ch >= '0' && ch <= '9') || ch == '_' || ch == '-' ||
                          ch == ':' || ch == '.';
        if (!safe) ch = '_';
    }
    return reason;
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

    const int total_goals = total_box_goals(level);
    if (total_goals == 0) {
        CBSSolver fallback;
        return fallback.solve(level, initial_state, heuristic);
    }

    // Stop after several waves that fail to improve the state. This prevents an
    // infinite loop on unschedulable or cyclic task sets while still allowing a
    // few local-repair attempts to unblock progress.
    constexpr int kNoProgressBudget = 50;
    // Convert the external millisecond budget to the seconds representation used
    // by chrono::duration<double>. The main loop checks this before every wave.
    const double kTimeBudgetSeconds = static_cast<double>(config_.planning_time_budget_ms) / 1000.0;
    const PlanningDeadline deadline = PlanningDeadline::after(std::chrono::milliseconds(config_.planning_time_budget_ms));

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
    TaskScheduler scheduler(config_);
    TaskPrioritizer prioritizer;
    LocalRepair repair(config_);
    PlanConflictRepairer conflict_repairer;
    int no_progress_iters = 0;
    int wave_count = 0;
    std::string stop_reason = "loop_not_started";
    const int max_waves = std::max(32, level.rows * level.cols * 4);
    WaveProgressGuard progress_guard;
    progress_guard.reset(level, current);
    int best_goals_completed = WaveProgressGuard::goals_completed(level, current);

    // Main HTN loop: each iteration plans one wave from the latest simulated
    // state. Inputs are the level and `current`; output is appended into
    // `accumulated` and reflected back into `current` via apply_prefix().
    while (true) {
        if (++wave_count > max_waves) {
            stop_reason = "max_waves";
            break;
        }
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - start_time).count();
        if (elapsed >= kTimeBudgetSeconds || deadline.expired()) {
            stop_reason = "deadline_before_wave";
            break;
        }

        // Convert unsatisfied goals into high-level delivery tasks. Each Task is
        // an intuitive contract: move a compatible box/agent from its source to
        // its target goal while respecting hospital-domain constraints.  On the
        // very first wave, try every immediately generatable direct delivery and
        // agent-goal task before expensive blocker enumeration; if it yields a
        // valid prefix, return useful work sooner and let the next wave perform
        // the deeper analysis from the advanced state.  Do not cap this by the
        // small default batch size: independent agents should get a chance to
        // start in the same opening wave.
        std::vector<Task> tasks;
        std::vector<AgentPlan> wave_agent_plans;
        Plan wave;
        bool used_cheap_first_prefix = false;
        if (accumulated.steps.empty()) {
            TaskGenerationOptions cheap_options;
            cheap_options.include_blocker_tasks = false;
            cheap_options.include_agent_goal_tasks = true;
            cheap_options.max_direct_delivery_tasks = 0;
            std::vector<Task> cheap_tasks = generator.generate_delivery_tasks(level, current, std::vector<AgentPlan>{}, deadline, cheap_options);
            if (cheap_tasks.size() > static_cast<std::size_t>(config_.max_batch_tasks) && !deadline.expired()) {
                std::vector<AgentPlan> cheap_agent_plans = scheduler.build_agent_plans(level, current, cheap_tasks, deadline);
                Plan cheap_wave = PlanMerger::merge_agent_plans(cheap_agent_plans, current.num_agents());
                PlanMerger::compact_independent_actions(level, current, cheap_wave);
                if (!cheap_wave.empty()) {
                    tasks = std::move(cheap_tasks);
                    wave_agent_plans = std::move(cheap_agent_plans);
                    wave = std::move(cheap_wave);
                    used_cheap_first_prefix = true;
                }
            }
        }

        if (wave.empty()) {
            tasks = generator.generate_delivery_tasks(level, current, std::vector<AgentPlan>{}, deadline);
        }
        if (kVerboseTasks) {
            // Scoring is done on a copy because trace output should explain the
            // scheduler's priorities without mutating the task list returned by
            // the generator.
            std::vector<Task> scored = tasks;
            prioritizer.score(level, current, scored);
            std::cerr << "[HTN] competitive_wave tasks=" << scored.size()
                      << " elapsed=" << elapsed << "s"
                      << (used_cheap_first_prefix ? " cheap_first_prefix=true" : "") << '\n';
            HTNTracePrinter::print_task_batch(scored, std::cerr);
            for (const auto& reason : generator.skip_reasons()) {
                std::cerr << "[HTN] " << reason << '\n';
            }
        }
        // No tasks means every currently generatable delivery goal is already
        // satisfied or impossible to assign, so there is no useful competitive
        // wave to append.
        if (tasks.empty()) {
            stop_reason = (WaveProgressGuard::goals_completed(level, current) >= total_goals)
                              ? "solved"
                              : "no_tasks";
            break;
        }

        // Ask the scheduler to turn high-level tasks into a concrete joint plan.
        // Input: current world state plus the candidate tasks. Output: a wave of
        // synchronized actions that can be appended to the final plan.
        if (deadline.expired()) {
            stop_reason = "deadline_before_scheduler";
            break;
        }
        if (wave.empty()) {
            wave_agent_plans = scheduler.build_agent_plans(level, current, tasks, deadline);
            wave = PlanMerger::merge_agent_plans(wave_agent_plans, current.num_agents());
            PlanMerger::compact_independent_actions(level, current, wave);
        }
        bool conflict_repair_already_attempted = false;
        if (wave.empty()) {
            // An empty wave means the scheduler could not safely reserve paths
            // for the generated tasks. Local repair now performs real low-level
            // planning alternatives (delays, alternate agents/parking, and a
            // relaxed-reservation replan) and promotes a successful repair into
            // the wave instead of repeatedly returning the same failed task.
            if (kVerboseTasks) {
                std::cerr << "[HTN] scheduler_empty; attempting local repair on ready tasks\n";
            }
            TaskPlan failed;
            failed.success = false;
            failed.failure_reason = "scheduler_empty";
            for (const Task& task : tasks) {
                if (deadline.expired()) {
                    stop_reason = "deadline_during_repair";
                    break;
                }
                const RepairResult repaired = repair.repair(level, current, task, failed, ReservationTable{}, 0, deadline);
                if (kVerboseTasks) {
                    std::cerr << "[HTN] repair outcome=" << static_cast<int>(repaired.outcome)
                              << " success=" << (repaired.plan.success ? "true" : "false")
                              << " reason=" << repaired.reason << '\n';
                }
                if (!repaired.plan.success) continue;
                if (repaired.outcome == RepairStageOutcome::SafePrefixFallback && !accumulated.steps.empty()) {
                    // Repair may validate that the already accumulated safe prefix is
                    // the best available output. Reset the stagnation counter so the
                    // solver can re-analyze from the unchanged state once more.
                    no_progress_iters = 0;
                    continue;
                }

                wave_agent_plans.assign(current.num_agents(), AgentPlan{});
                for (int agent = 0; agent < current.num_agents(); ++agent) {
                    wave_agent_plans[static_cast<std::size_t>(agent)].agent = agent;
                    wave_agent_plans[static_cast<std::size_t>(agent)].positions = {current.agent_positions[static_cast<std::size_t>(agent)]};
                }
                if (repaired.plan.agent_id >= 0 && repaired.plan.agent_id < current.num_agents()) {
                    wave_agent_plans[static_cast<std::size_t>(repaired.plan.agent_id)] = repaired.plan.agent_plan;
                    wave = PlanMerger::merge_agent_plans(wave_agent_plans, current.num_agents());
                    PlanMerger::compact_independent_actions(level, current, wave);
                    break;
                }
            }

            // If the task-level local repair could not promote a single repaired
            // task, still give the CBS-style conflict repairer the scheduler's
            // original per-agent timelines.  Those plans may be unmergeable only
            // because of a temporal conflict; the conflict repairer can identify
            // the involved agents and replan/delay them without discarding the
            // whole scheduler wave.
            if (wave.empty() && !deadline.expired() && has_planned_actions(wave_agent_plans)) {
                conflict_repair_already_attempted = true;
                const PlanConflictRepairer::Result repaired_wave = conflict_repairer.repair(level, current, wave_agent_plans);
                if (repaired_wave.conflict_free && !repaired_wave.plan.empty()) {
                    if (kVerboseTasks) {
                        std::cerr << "[HTN] cbs_style_repair recovered scheduler_empty iterations="
                                  << repaired_wave.iterations << " steps=" << repaired_wave.plan.steps.size() << '\n';
                    }
                    wave_agent_plans = repaired_wave.agent_plans;
                    wave = PlanMerger::merge_agent_plans(wave_agent_plans, current.num_agents());
                    PlanMerger::compact_independent_actions(level, current, wave);
                } else if (kVerboseTasks) {
                    std::cerr << "[HTN] cbs_style_repair could not recover scheduler_empty wave\n";
                }
            }

            if (wave.empty() && ++no_progress_iters >= kNoProgressBudget) {
                if (stop_reason.empty() || stop_reason == "loop_not_started") {
                    stop_reason = deadline.expired() ? "deadline_during_repair" : "scheduler_empty";
                }
                break;
            }
        }

        // The scheduler normally uses reservations, but a competitive wave can
        // still expose a conflict once the selected task/agent timelines are
        // composed. Repair operates on the AgentPlan wave directly so it can add
        // conflict constraints and replan alternate routes without changing task
        // priorities or reshuffling the accepted schedule.
        // for (JointAction ja : wave.steps) std::cerr << ja.to_string() << std::endl;
        if (deadline.expired()) {
            stop_reason = "deadline_before_conflict_repair";
            break;
        }
        if (!conflict_repair_already_attempted) {
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
        }

        // Simulate the wave before accepting it.  A locally executable wave can
        // still be cyclic (for example Push(E,E) immediately followed by the
        // inverse Pull(W,W)) or return to a previously accepted world.  Reject
        // those waves before appending them so cyclic benchmark cases terminate
        // with a compact diagnostic prefix instead of thousands of oscillations.
        const WaveProgressDecision progress = progress_guard.assess(level, current, wave, best_goals_completed);
        if (!progress.accept) {
            if (kVerboseTasks) {
                std::cerr << "[HTN] " << progress.reason << "; rejecting wave steps=" << wave.steps.size() << '\n';
            }
            if (++no_progress_iters >= kNoProgressBudget) {
                stop_reason = "stagnation_guard:" + compact_stop_reason(progress.reason);
                break;
            }
            continue;
        }

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
        progress_guard.remember_accepted(current);
        if (WaveProgressGuard::goals_completed(level, current) >= total_goals) {
            stop_reason = "solved";
            break;
        }
        if (progress.completed_new_goal || (progress.moved_any_box && progress.goals_after >= best_goals_completed)) {
            best_goals_completed = std::max(best_goals_completed, progress.goals_after);
            no_progress_iters = 0;
        } else if (++no_progress_iters >= kNoProgressBudget) {
            stop_reason = "stagnation_guard:no_goal_progress";
            break;
        }
    }

    // Return the best HTN/reservation prefix immediately.  The old CBS fallback
    // had no wall-clock budget and could consume the remaining server time after
    // the competitive solver had already found useful work, turning partial
    // failures into hard timeouts.  If that prefix is still incomplete, emit a
    // stable marker so benchmark triage can separate "returned a useful but
    // unfinished plan" from zero-action task-generation failures.
    const int completed_goals = WaveProgressGuard::goals_completed(level, current);
    if (!accumulated.steps.empty() && completed_goals < total_goals) {
        std::cerr << "[HTN] partial_plan_unsolved stop_reason=" << compact_stop_reason(stop_reason)
                  << " steps=" << accumulated.steps.size()
                  << " goals_completed=" << completed_goals << '/' << total_goals
                  << " waves=" << wave_count
                  << " no_progress_iters=" << no_progress_iters << '\n';
    }
    return accumulated;
}
