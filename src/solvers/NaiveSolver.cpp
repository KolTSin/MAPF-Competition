#include "solvers/NaiveSolver.hpp"

#include "actions/Action.hpp"
#include "actions/JointAction.hpp"
#include "search/AStar.hpp"
#include "plan/PlanMerger.hpp"
#include "search/heuristics/Heuristic.hpp"

#include <iostream>

Plan NaiveSolver::solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) {
    (void) level;
    int num_agents = initial_state.num_agents();
    AStar astar(heuristic);
    std::vector<std::vector<Action>> agent_plans(num_agents);
    
    std::vector<Action> plan = astar.search(level, initial_state, 0);
    for(int agent = 0 ; agent < initial_state.num_agents() ; agent++) {
        std::cerr << "this is the agent id: " << agent <<'\n';
        plan = astar.search(level, initial_state, agent);
        agent_plans[agent] = plan;
    }
    
    return PlanMerger::merge_agent_plans(agent_plans, num_agents);
}