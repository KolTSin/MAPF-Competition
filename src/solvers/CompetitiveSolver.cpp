#include "solvers/CompetitiveSolver.hpp"

#include "actions/ActionSemantics.hpp"
#include "analysis/LevelAnalyzer.hpp"
#include "hospital/LocalRepair.hpp"
#include "plan/AgentPlan.hpp"
#include "plan/PlanConflictRepairer.hpp"
#include "plan/PlanMerger.hpp"
#include "solvers/SequentialSolver.hpp"
#include "tasks/HTNTracePrinter.hpp"
#include "tasks/TaskGenerator.hpp"
#include "tasks/TaskPrioritizer.hpp"
#include "tasks/TaskScheduler.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>

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
// Returning true keeps the solver explainable by default: each wave can print
// which tasks were generated, how they were scored, and why goals were skipped.

std::vector<AgentPlan> agent_plans_from_plan(const Plan& plan, const State& initial_state) {
    std::vector<AgentPlan> agent_plans(initial_state.num_agents());
    for (int agent = 0; agent < initial_state.num_agents(); ++agent) {
        AgentPlan& agent_plan = agent_plans[static_cast<std::size_t>(agent)];
        agent_plan.agent = agent;
        Position cur = initial_state.agent_positions[static_cast<std::size_t>(agent)];
        agent_plan.positions.push_back(cur);

        for (const JointAction& step : plan.steps) {
            Action action = Action::noop();
            if (agent < static_cast<int>(step.actions.size())) {
                action = step.actions[static_cast<std::size_t>(agent)];
            }
            agent_plan.actions.push_back(action);
            cur = ActionSemantics::compute_effect(cur, action).agent_to;
            agent_plan.positions.push_back(cur);
        }
    }

    return agent_plans;
}

bool configured_verbose_tasks() {
    if (const char* v = std::getenv("MAPF_VERBOSE_TASKS")) {
        return std::atoi(v) != 0;
    }
    return true;
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
    //   1. analyze static/dynamic map structure,
    //   2. generate high-level delivery tasks,
    //   3. score them for traceability,
    //   4. schedule primitive actions with reservations, and
    //   5. try targeted repair if scheduling fails.
    LevelAnalyzer analyzer;
    TaskGenerator generator;
    TaskScheduler scheduler;
    TaskPrioritizer prioritizer;
    LocalRepair repair;
    PlanConflictRepairer conflict_repairer;
    int no_progress_iters = 0;

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

    // Main HTN loop: each iteration plans one wave from the latest simulated
    // state. Inputs are the level and `current`; output is appended into
    // `accumulated` and reflected back into `current` via apply_prefix().
    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - start_time).count();
        if (elapsed >= kTimeBudgetSeconds) break;

        // Analyze the map at this state before task generation. The result is
        // currently computed for side effects/caches used by downstream hospital
        // modules; the local variable is not otherwise needed in this method.
        (void)analyzer.analyze(level, current);

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
        // still be treated as a naive HLA batch: first build each selected
        // task/agent timeline, then run a bounded CBS-style delay repair over
        // the merged joint actions before committing anything to the answer.
        // for (JointAction ja : wave.steps) std::cerr << ja.to_string() << std::endl;
        const PlanConflictRepairer::Result repaired_wave = conflict_repairer.repair(level, current, wave);
        if (repaired_wave.conflict_free) {
            if (kVerboseTasks && repaired_wave.changed) {
                std::cerr << "[HTN] cbs_style_repair inserted waits iterations="
                          << repaired_wave.iterations << " steps=" << repaired_wave.plan.steps.size() << '\n';
            }
            wave = repaired_wave.plan;
            wave_agent_plans = agent_plans_from_plan(wave, current);
        } else if (kVerboseTasks) {
            std::cerr << "[HTN] cbs_style_repair could not prove conflict-free wave; using scheduler wave\n";
        }

        // Snapshot state before committing the wave so we can determine whether
        // it made meaningful progress after simulation.
        const State before = current;
        const int goals_before = goals_completed(before);

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
        // Treat either a newly completed box goal or an agent movement as
        // progress. Agent-only motion can be necessary setup for later box
        // deliveries, so it should not be mistaken for a stalled iteration.
        const bool progressed = (goals_after > goals_before) || !(current.agent_positions == before.agent_positions);
        if (progressed) {
            no_progress_iters = 0;
        } else if (++no_progress_iters >= kNoProgressBudget) {
            break;
        }
    }

   // If the competitive pipeline produced no executable actions, or if it only
    // made partial progress before getting stuck, return a simpler complete-solver
    // attempt. This preserves a useful answer for small levels where sequential
    // planning is sufficient and prevents a partial HLA prefix from masquerading
    // as a complete solution.
    const auto all_goals_satisfied = [&level](const State& s) {
        for (int r = 0; r < level.rows; ++r) {
            for (int c = 0; c < level.cols; ++c) {
                const char g = level.goal_at(r, c);
                if (g >= 'A' && g <= 'Z' && s.box_at(r, c) != g) return false;
                if (g >= '0' && g <= '9') {
                    const int agent = g - '0';
                    if (agent >= s.num_agents() || !(s.agent_positions[agent] == Position{r, c})) return false;
                }
            }
        }
        return true;
    };
    if (accumulated.empty() || !all_goals_satisfied(current)) {
        SequentialSolver fallback;
        Plan fallback_plan = fallback.solve(level, initial_state, heuristic);
        if (!fallback_plan.empty()) return fallback_plan;
    }

    return accumulated;
}
