#include "solvers/SequentialSolver.hpp"

#include "actions/Action.hpp"
#include "actions/JointAction.hpp"
#include "hospital/BoxTransportPlanner.hpp"
#include "tasks/HTNTracePrinter.hpp"
#include "tasks/TaskGenerator.hpp"

#include <iostream>

Plan SequentialSolver::solve(const Level& level, const State& initial_state, const IHeuristic&) {
    const int num_agents = initial_state.num_agents();
    std::vector<Action> primitive_actions;

    TaskGenerator task_generator;
    const std::vector<Task> tasks = task_generator.generate_delivery_tasks(level, initial_state);
    HTNTracePrinter::print_task_batch(tasks, std::cerr);
    for (const auto& reason : task_generator.skip_reasons()) {
        std::cerr << "[HTN] " << reason << '\n';
    }

    if (!tasks.empty()) {
        const Task& task = tasks.front();
        BoxTransportPlanner planner;
        TaskPlan task_plan = planner.plan(level, initial_state, task);
        if (task_plan.success) {
            primitive_actions = task_plan.primitive_actions;
            std::cerr << "[HTN] picked " << task.describe() << " primitive_actions=" << primitive_actions.size() << '\n';
        } else {
            std::cerr << "[HTN] plan failed for " << task.describe() << " reason=" << task_plan.failure_reason << '\n';
        }
    }

    Plan result;
    for (const Action& a : primitive_actions) {
        JointAction ja;
        ja.actions.assign(num_agents, Action::noop());
        if (num_agents > 0 && !tasks.empty()) {
            ja.actions[tasks.front().agent_id] = a;
        }
        result.steps.push_back(std::move(ja));
    }
    return result;
}
