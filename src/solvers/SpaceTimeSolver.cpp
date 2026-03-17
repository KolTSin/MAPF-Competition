#include "solvers/SpaceTimeSolver.hpp"

#include "actions/Action.hpp"
#include "actions/JointAction.hpp"
#include "search/SpaceTimeAStar.hpp"
#include "plan/PlanMerger.hpp" 
#include "plan/ReservationTable.hpp"

#include <iostream>

Plan SpaceTimeSolver::solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) {
    int num_agents = initial_state.num_agents();

    std::vector<std::vector<Action>> agent_plans(num_agents);

    ReservationTable reservations;
    SpaceTimeAStar astar(heuristic);

    std::cerr << "SpaceTimeSolver start, num_agents = " << num_agents << '\n';

    for (int agent = 0; agent < num_agents; ++agent) {
        std::cerr << "Planning for agent " << agent << "...\n";

        Position start = initial_state.agent_positions[agent];
        Position goal;

        try {
            agent_plans[agent] = astar.search(level, start, goal, reservations);
            std::cerr << "Agent " << agent << " plan length = "
                    << agent_plans[agent].size() << '\n';
        } catch (const std::exception& e) {
            std::cerr << "AStar failed for agent " << agent
                    << " with exception: " << e.what() << '\n';
            throw;
        }

        std::cerr << "Finished agent " << agent << '\n';
    }

    std::cerr << "About to merge plans\n";
    Plan result = PlanMerger::merge_agent_plans(agent_plans, num_agents);
    for (const JointAction& ja : result.steps){
        std::cerr << ja.to_string() << '\n';
    }

    return result;
}