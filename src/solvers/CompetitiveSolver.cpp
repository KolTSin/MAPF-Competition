#include "solvers/CompetitiveSolver.hpp"

#include "analysis/LevelAnalyzer.hpp"
#include "tasks/TaskGenerator.hpp"
#include "tasks/TaskScheduler.hpp"

Plan CompetitiveSolver::solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) {
    (void)heuristic;
    LevelAnalyzer analyzer;
    (void)analyzer.analyze(level, initial_state);

    TaskGenerator generator;
    std::vector<Task> tasks = generator.generate_delivery_tasks(level, initial_state);

    TaskScheduler scheduler;
    return scheduler.build_plan(level, initial_state, tasks);
}
