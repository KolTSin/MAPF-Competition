#include "solvers/LNSHTNSolver.hpp"

#include "actions/ActionApplicator.hpp"
#include "actions/ActionLibrary.hpp"
#include "actions/ActionSemantics.hpp"
#include "plan/ConflictDetector.hpp"
#include "plan/LNSRepairer.hpp"
#include "plan/PlanMerger.hpp"
#include "plan/WaveProgressGuard.hpp"
#include "tasks/HTNTracePrinter.hpp"
#include "tasks/TaskGenerator.hpp"
#include "tasks/TaskPrioritizer.hpp"
#include "tasks/TaskScheduler.hpp"
#include "utils/PlanningDeadline.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

void apply_prefix(State& state, const Plan& plan, int horizon) {
    const int n = std::min(horizon, static_cast<int>(plan.steps.size()));
    for (int t = 0; t < n; ++t) {
        const JointAction& ja = plan.steps[static_cast<std::size_t>(t)];
        for (int a = 0; a < state.num_agents() && a < static_cast<int>(ja.actions.size()); ++a) {
            const ActionEffect eff = ActionSemantics::compute_effect(state.agent_positions[static_cast<std::size_t>(a)], ja.actions[static_cast<std::size_t>(a)]);
            state.agent_positions[static_cast<std::size_t>(a)] = eff.agent_to;
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

bool verbose_lns() {
    if (const char* v = std::getenv("MAPF_VERBOSE_TASKS")) return std::atoi(v) != 0;
    return false;
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

int manhattan(Position a, Position b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

Position preferred_target_for_agent(const State& state, const std::vector<Task>& tasks, int agent) {
    Position best{-1, -1};
    int best_score = std::numeric_limits<int>::max();
    for (const Task& task : tasks) {
        if (task.agent_id != agent) continue;
        const Position from = state.agent_positions[static_cast<std::size_t>(agent)];
        const Position target = (task.box_id >= 'A' && task.box_id <= 'Z') ? task.box_pos : task.goal_pos;
        if (target.row < 0 || target.col < 0) continue;
        const int score = manhattan(from, target) - task.priority;
        if (score < best_score) {
            best_score = score;
            best = target;
        }
    }
    return best;
}

std::vector<Action> ranked_one_step_actions(const State& state, const std::vector<Task>& tasks, int agent) {
    std::vector<Action> actions(ActionLibrary::ALL_ACTIONS.begin(), ActionLibrary::ALL_ACTIONS.end());
    const Position target = preferred_target_for_agent(state, tasks, agent);
    const Position from = state.agent_positions[static_cast<std::size_t>(agent)];
    std::stable_sort(actions.begin(), actions.end(), [&](const Action& lhs, const Action& rhs) {
        const ActionEffect le = ActionSemantics::compute_effect(from, lhs);
        const ActionEffect re = ActionSemantics::compute_effect(from, rhs);
        const int ld = (target.row < 0) ? 0 : manhattan(le.agent_to, target);
        const int rd = (target.row < 0) ? 0 : manhattan(re.agent_to, target);
        if (ld != rd) return ld < rd;
        return static_cast<int>(lhs.type) < static_cast<int>(rhs.type);
    });
    actions.push_back(Action::noop());
    return actions;
}

Plan pibt_style_fallback_step(const Level& level, const State& state, const std::vector<Task>& tasks) {
    const int n = state.num_agents();
    std::vector<int> order(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) order[static_cast<std::size_t>(i)] = i;
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
        const Position ta = preferred_target_for_agent(state, tasks, a);
        const Position tb = preferred_target_for_agent(state, tasks, b);
        const int da = (ta.row < 0) ? std::numeric_limits<int>::max() / 4 : manhattan(state.agent_positions[static_cast<std::size_t>(a)], ta);
        const int db = (tb.row < 0) ? std::numeric_limits<int>::max() / 4 : manhattan(state.agent_positions[static_cast<std::size_t>(b)], tb);
        return da < db;
    });

    JointAction joint;
    joint.actions.resize(static_cast<std::size_t>(n), Action::noop());
    for (int agent : order) {
        for (const Action& action : ranked_one_step_actions(state, tasks, agent)) {
            if (!ActionApplicator::is_applicable(level, state, agent, action)) continue;
            JointAction trial = joint;
            trial.actions[static_cast<std::size_t>(agent)] = action;
            Plan one;
            one.steps.push_back(trial);
            if (!ConflictDetector::has_conflict(one, state)) {
                joint = std::move(trial);
                break;
            }
        }
    }

    Plan out;
    if (!std::all_of(joint.actions.begin(), joint.actions.end(), [](const Action& action) { return action.type == ActionType::NoOp; })) {
        out.steps.push_back(std::move(joint));
    }
    return out;
}

} // namespace

LNSHTNSolver::LNSHTNSolver(SolverConfig config) : config_(config) {}

Plan LNSHTNSolver::solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) {
    (void)heuristic;

    const int total_goals = total_box_goals(level);
    const PlanningDeadline deadline = PlanningDeadline::after(std::chrono::milliseconds(config_.planning_time_budget_ms));
    const auto start_time = std::chrono::steady_clock::now();
    const double budget_seconds = static_cast<double>(config_.planning_time_budget_ms) / 1000.0;
    const bool trace = verbose_lns();

    State current = initial_state;
    Plan accumulated;
    WaveProgressGuard progress_guard;
    progress_guard.reset(level, current);
    int best_goals_completed = WaveProgressGuard::goals_completed(level, current);

    TaskGenerator generator;
    TaskPrioritizer prioritizer;
    TaskScheduler scheduler;
    LNSRepairer lns;

    int no_progress_iters = 0;
    int wave_count = 0;
    std::string stop_reason = "loop_not_started";
    const int max_waves = std::max(32, level.rows * level.cols * 4);

    while (true) {
        if (++wave_count > max_waves) { stop_reason = "max_waves"; break; }
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        if (elapsed >= budget_seconds || deadline.expired()) { stop_reason = "deadline_before_wave"; break; }

        std::vector<Task> tasks = generator.generate_delivery_tasks(level, current, std::vector<AgentPlan>{}, deadline);
        prioritizer.score(level, current, tasks);
        if (trace) {
            std::cerr << "[HTN-LNS] wave=" << wave_count << " tasks=" << tasks.size() << " elapsed=" << elapsed << "s\n";
            HTNTracePrinter::print_task_batch(tasks, std::cerr);
        }
        if (tasks.empty()) {
            stop_reason = (WaveProgressGuard::goals_completed(level, current) >= total_goals) ? "solved" : "no_tasks";
            break;
        }

        std::vector<AgentPlan> wave_agent_plans = scheduler.build_agent_plans(level, current, tasks, deadline);
        Plan wave = PlanMerger::merge_agent_plans(wave_agent_plans, current.num_agents());
        PlanMerger::compact_independent_actions(level, current, wave);

        const int conflicts_before_lns = static_cast<int>(ConflictDetector::findAllConflicts(wave, current, false).size());
        if (!wave.empty() && conflicts_before_lns > 0 && !deadline.expired()) {
            const LNSRepairer::Result repaired = lns.repair_wave(level, current, tasks, wave_agent_plans, deadline);
            if (trace) {
                std::cerr << "[HTN-LNS] repair conflicts " << repaired.conflicts_before << " -> "
                          << repaired.conflicts_after << " iterations=" << repaired.iterations << '\n';
            }
            if (repaired.changed && repaired.conflicts_after <= conflicts_before_lns) {
                wave_agent_plans = repaired.agent_plans;
                wave = PlanMerger::merge_agent_plans(wave_agent_plans, current.num_agents());
                PlanMerger::compact_independent_actions(level, current, wave);
            }
        }

        if (wave.empty() || ConflictDetector::has_conflict(wave, current)) {
            Plan fallback = pibt_style_fallback_step(level, current, tasks);
            if (!fallback.empty()) {
                if (trace) std::cerr << "[HTN-LNS] using PIBT-style one-step fallback\n";
                wave = std::move(fallback);
            }
        }

        if (wave.empty()) {
            if (++no_progress_iters >= 12) { stop_reason = "empty_wave"; break; }
            continue;
        }

        const WaveProgressDecision progress = progress_guard.assess(level, current, wave, best_goals_completed);
        if (!progress.accept) {
            if (trace) std::cerr << "[HTN-LNS] rejected wave: " << progress.reason << '\n';
            Plan fallback = pibt_style_fallback_step(level, current, tasks);
            if (fallback.empty()) {
                if (++no_progress_iters >= 12) { stop_reason = "stagnation_guard:" + compact_stop_reason(progress.reason); break; }
                continue;
            }
            wave = std::move(fallback);
        }

        for (const JointAction& step : wave.steps) accumulated.steps.push_back(step);
        apply_prefix(current, wave, static_cast<int>(wave.steps.size()));
        progress_guard.remember_accepted(current);

        const int completed = WaveProgressGuard::goals_completed(level, current);
        if (completed >= total_goals) { stop_reason = "solved"; break; }
        if (completed > best_goals_completed || progress.moved_any_box) {
            best_goals_completed = std::max(best_goals_completed, completed);
            no_progress_iters = 0;
        } else if (++no_progress_iters >= 12) {
            stop_reason = "stagnation_guard:no_goal_progress";
            break;
        }
    }

    const int completed_goals = WaveProgressGuard::goals_completed(level, current);
    if (!accumulated.steps.empty() && completed_goals < total_goals) {
        std::cerr << "[HTN-LNS] partial_plan_unsolved stop_reason=" << compact_stop_reason(stop_reason)
                  << " steps=" << accumulated.steps.size()
                  << " goals_completed=" << completed_goals << '/' << total_goals
                  << " waves=" << wave_count << '\n';
    }
    return accumulated;
}
