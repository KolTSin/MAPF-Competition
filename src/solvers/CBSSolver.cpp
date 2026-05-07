#include "solvers/CBSSolver.hpp"

#include "actions/Action.hpp"
#include "actions/JointAction.hpp"
#include "search/AStar.hpp"
#include "search/SpaceTimeAStar.hpp"
#include "search/SpaceTimeAStar.hpp"
#include "search/CBSNode.hpp"
#include "search/CBS.hpp"
#include "plan/PlanMerger.hpp"
#include "plan/AgentPlan.hpp"
#include "search/heuristics/Heuristic.hpp"
#include "plan/ConstraintTable.hpp"
#include "plan/ReservationTable.hpp"
#include "plan/ConflictDetector.hpp"
#include "search/CBSFrontier.hpp"

#include <iostream>

Plan CBSSolver::solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) {
    (void) level;

    int num_agents = initial_state.num_agents();

    SpaceTimeAStar stastar(heuristic);
    AStar astar(heuristic);
    // CBS stastar(heuristic);

    std::vector<AgentPlan> agent_plans(num_agents);
    AgentPlan plan;
    ReservationTable conflict_table;
    

    std::vector<CBSNode> CT;
    CT.reserve(2048);
    CBSNode start;
    std::vector<Constraint> constraint;

    CBSFrontier open(&CT);

    // initial plan for all agents individually
    for (int agent = 0; agent < num_agents; agent++){
        plan = stastar.search(level, initial_state, agent, 20'000, conflict_table); //, 20000, conflict_table);
        agent_plans[agent] = plan;
    }

    start.plans = agent_plans;
    CT.push_back(start);
    open.push(0);

    // go through the CT until a solution is found
    while (!open.empty()){
        const int current_index = open.pop();
        const CBSNode current = CT[current_index];
        // std::cerr << "current index: " << current_index << '\n';

        // detect conflicts
        ConflictDetector conflict_detector;
        Conflict conflict = conflict_detector.findFirstConflict(current.plans);
        // std::cerr << "the conflict found: " << conflict.agents[0] << std::endl;
        // std::cerr << conflict.to_string() << std::endl;
        if (conflict.agents[0] == -1){
            // std::cerr << "no conflicts found, current index: " << current_index << '\n'
            //     << conflict.to_string() << std::endl; 
            return PlanMerger::merge_agent_plans(current.plans, num_agents);
        }
        

        for (int i = 0 ; i < 2; i++){
            std::cerr << "creating a new CBSNode" << std::endl;
            // create a new node
            CBSNode a;
            a.constraints = current.constraints;
            a.plans = current.plans;
            int makespan = 0;
            for (const auto& p : a.plans) {
                makespan = std::max(makespan, static_cast<int>(p.actions.size()));
            }
            a.makespan = makespan;

            std::cerr << "creating a new constraint" << std::endl;
            // add the new constraint
            Constraint c;
            c.agent_id = conflict.agents[i];
            c.time = conflict.time;
            c.cell = conflict.cell;
            c.from = conflict.from;
            c.to = conflict.to;
            a.constraints.push_back(c);
            // std::cerr << "constraint created!" << std::endl;

            // add the current constraints into the reservations table
            ReservationTable res_table;
            for (Constraint cons : a.constraints){
                if (cons.agent_id == conflict.agents[i]){
                    std::cerr << cons.to_string() << std::endl;
                    // std::cerr << "reserving the cells/edges in the constraints" << std::endl;
                    res_table.reserve_cell(cons.cell.row,cons.cell.col,cons.time,-1);
                    res_table.reserve_edge(cons.from[0], cons.to[0], cons.time + 1,-1);
                    res_table.reserve_edge(cons.from[1], cons.to[1], cons.time,-1);
                }
            }
            std::cerr << "restarting search for agent " << conflict.agents[i] << std::endl;
            // search for the constrained agent
            plan = stastar.search(level, initial_state, conflict.agents[i], 20'000, res_table);
            // std::cerr << "search for agent " << conflict.agents[i] << " complete" << std::endl;
            res_table.clear_reservations();
            // std::cerr << "reservations cleared!" << std::endl;
            if (!plan.valid()){
                continue;
            }
            if (plan == a.plans[conflict.agents[i]]){
                continue;
            }
            a.plans[conflict.agents[i]] = plan;

            // push the node into open
            // std::cerr << "pushing the node into open" << std::endl;
            CT.push_back(a);
            int next_index = static_cast<int>(CT.size()) - 1;
            open.push(next_index);
            // std::cerr << "done!" << std::endl;
        }
    }

    return PlanMerger::merge_agent_plans(agent_plans, num_agents);
}