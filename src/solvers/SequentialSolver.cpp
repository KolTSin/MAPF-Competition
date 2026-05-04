#include "solvers/SequentialSolver.hpp"

#include "tasks/HTNTracePrinter.hpp"
#include "tasks/TaskGenerator.hpp"
#include "tasks/TaskScheduler.hpp"

#include <iostream>

Plan SequentialSolver::solve(const Level& level, const State& initial_state, const IHeuristic&) {
    TaskGenerator task_generator;
    const std::vector<Task> tasks = task_generator.generate_delivery_tasks(level, initial_state);
    HTNTracePrinter::print_task_batch(tasks, std::cerr);
    for (const auto& reason : task_generator.skip_reasons()) {
        std::cerr << "[HTN] " << reason << '\n';
    }

    TaskScheduler scheduler;
    return scheduler.build_plan(level, initial_state, tasks);
}
