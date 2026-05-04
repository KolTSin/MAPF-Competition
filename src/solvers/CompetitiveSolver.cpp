#include "solvers/CompetitiveSolver.hpp"

#include "actions/ActionSemantics.hpp"
#include "analysis/LevelAnalyzer.hpp"
#include "tasks/TaskGenerator.hpp"
#include "tasks/TaskScheduler.hpp"

#include <algorithm>

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
}

Plan CompetitiveSolver::solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) {
    (void)heuristic;
    constexpr int kIterationBudget = 20;
    constexpr int kSafePrefixHorizon = 6;

    State current = initial_state;
    Plan accumulated;

    LevelAnalyzer analyzer;
    TaskGenerator generator;
    TaskScheduler scheduler;

    for (int iter = 0; iter < kIterationBudget; ++iter) {
        (void)analyzer.analyze(level, current);
        std::vector<Task> tasks = generator.generate_delivery_tasks(level, current);
        if (tasks.empty()) break;

        Plan wave = scheduler.build_plan(level, current, tasks);
        if (wave.empty()) break;

        const int safe_prefix = std::max(1, std::min(kSafePrefixHorizon, static_cast<int>(wave.steps.size())));
        for (int i = 0; i < safe_prefix; ++i) {
            accumulated.steps.push_back(wave.steps[i]);
        }
        apply_prefix(current, wave, safe_prefix);
    }

    return accumulated;
}
