#include "solvers/SpaceTimeSolver.hpp"

#include "actions/Action.hpp"
#include "actions/JointAction.hpp"
#include "search/SpaceTimeAStar.hpp"
#include "plan/PlanMerger.hpp" 
#include "plan/ReservationTable.hpp"

#include <iostream>

Plan SpaceTimeSolver::solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) {
    int num_agents = initial_state.num_agents();

    std::vector<AgentPlan> agent_plans(num_agents);

    ReservationTable reservations;
    SpaceTimeAStar astar(heuristic);
    int max_time = 20'000;

    std::cerr << "SpaceTimeSolver start, num_agents = " << num_agents << '\n';
    for (int agent=0; agent < num_agents; ++agent){
        reservations.reserve_cell(initial_state.agent_positions[agent].row,initial_state.agent_positions[agent].col,0, agent);
    }

    for (int agent = 0; agent < num_agents; ++agent) {
        std::cerr << "Planning for agent " << agent << "...\n";


        try {
            std::cerr << "starting now.." << '\n';
            agent_plans[agent] = astar.search(level, initial_state, agent, max_time, reservations);
            reservations.reserve_path(agent_plans[agent],initial_state.agent_positions[agent],agent);
            
            std::cerr << "Agent " << agent << " plan length = "
                    << agent_plans[agent].actions.size() << '\n';
        } catch (const std::exception& e) {
            std::cerr << "AStar failed for agent " << agent
                    << " with exception: " << e.what() << '\n';
            throw;
        }

        std::cerr << "Finished agent " << agent << '\n';
    }

    std::cerr << "About to merge plans\n";
    Plan result = PlanMerger::merge_agent_plans(agent_plans, num_agents);
    // for (const JointAction& ja : result.steps){
    //     std::cerr << ja.to_string() << '\n';
    // }

    return result;
}