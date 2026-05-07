#include "solvers/SequentialSolver.hpp"

#include "tasks/HTNTracePrinter.hpp"
#include "tasks/TaskGenerator.hpp"
#include "tasks/TaskScheduler.hpp"
#include "search/AStar.hpp"
#include "plan/PlanMerger.hpp"

#include <iostream>

Plan SequentialSolver::solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) {
    int num_agents = initial_state.num_agents();

    std::vector<AgentPlan> agent_plans(num_agents);

    AStar astar(heuristic);

    std::cerr << "SequentialSolver start, num_agents = " << num_agents << '\n';

    for (int agent = 0; agent < num_agents; ++agent) {
        std::cerr << "Planning for agent " << agent << "...\n";

        try {
            agent_plans[agent] = astar.search(level, initial_state, agent);
            std::cerr << "Agent " << agent << " plan length = "
                    << agent_plans[agent].actions.size() << '\n';
        } catch (const std::exception& e) {
            std::cerr << "AStar failed for agent " << agent
                    << " with exception: " << e.what() << '\n';
            throw;
        }

        std::cerr << "Finished agent " << agent << '\n';
    }

    return PlanMerger::merge_agent_plans(agent_plans, num_agents);
}
