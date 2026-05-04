#include "solvers/CompetitiveSolver.hpp"

#include "actions/ActionSemantics.hpp"
#include "analysis/LevelAnalyzer.hpp"
#include "hospital/LocalRepair.hpp"
#include "tasks/TaskGenerator.hpp"
#include "tasks/TaskScheduler.hpp"

#include <algorithm>
#include <cstdlib>

namespace {
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
int configured_safe_prefix_horizon() {
    if (const char* v = std::getenv("MAPF_SAFE_PREFIX_HORIZON")) {
        const int parsed = std::atoi(v);
        if (parsed > 0) return parsed;
    }
    return 6;
}
}

Plan CompetitiveSolver::solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) {
    (void)heuristic;
    constexpr int kIterationBudget = 20;
    constexpr int kNoProgressBudget = 5;
    const int kSafePrefixHorizon = configured_safe_prefix_horizon();

    State current = initial_state;
    Plan accumulated;

    LevelAnalyzer analyzer;
    TaskGenerator generator;
    TaskScheduler scheduler;
    LocalRepair repair;
    int no_progress_iters = 0;
    const auto goals_completed = [&level](const State& s) {
        int done = 0;
        for (int r = 0; r < level.rows; ++r) for (int c = 0; c < level.cols; ++c) {
            const char g = level.goal_at(r, c);
            if (g >= 'A' && g <= 'Z' && s.box_at(r, c) == g) ++done;
        }
        return done;
    };

    for (int iter = 0; iter < kIterationBudget; ++iter) {
        (void)analyzer.analyze(level, current);
        std::vector<Task> tasks = generator.generate_delivery_tasks(level, current);
        if (tasks.empty()) break;

        Plan wave = scheduler.build_plan(level, current, tasks);
        if (wave.empty()) {
            TaskPlan failed;
            failed.success = false;
            failed.failure_reason = "scheduler_empty";
            const RepairResult repaired = repair.repair(level, current, tasks.front(), failed);
            if (repaired.plan.success && repaired.outcome == RepairStageOutcome::SafePrefixFallback && !accumulated.steps.empty()) {
                no_progress_iters = 0;
                continue;
            }
            if (++no_progress_iters >= kNoProgressBudget) break;
            continue;
        }
        const State before = current;
        const int goals_before = goals_completed(before);

        const int safe_prefix = std::max(1, std::min(kSafePrefixHorizon, static_cast<int>(wave.steps.size())));
        for (int i = 0; i < safe_prefix; ++i) {
            accumulated.steps.push_back(wave.steps[i]);
        }
        apply_prefix(current, wave, safe_prefix);
        const int goals_after = goals_completed(current);
        const bool progressed = (goals_after > goals_before) || !(current.agent_positions == before.agent_positions);
        if (progressed) {
            no_progress_iters = 0;
        } else if (++no_progress_iters >= kNoProgressBudget) {
            break;
        }
    }

    return accumulated;
}
