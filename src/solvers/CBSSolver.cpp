#include "solvers/CBSSolver.hpp"

#include "actions/Action.hpp"
#include "actions/JointAction.hpp"
#include "search/AStar.hpp"
#include "search/SpaceTimeAStar.hpp"
#include "search/CBSNode.hpp"
#include "plan/PlanMerger.hpp"
#include "plan/AgentPlan.hpp"
#include "search/heuristics/Heuristic.hpp"
#include "plan/ConstraintTable.hpp"
#include "plan/ReservationTable.hpp"
#include "plan/ConflictDetector.hpp"
#include "search/CBSFrontier.hpp"

#include <algorithm>
#include <iostream>

namespace {

void update_node_costs(CBSNode& node) {
    node.makespan = 0;
    node.sum_of_costs = 0;
    for (const auto& plan : node.plans) {
        node.makespan = std::max(node.makespan, static_cast<int>(plan.actions.size()));
        node.sum_of_costs += plan.cost();
    }
}

void add_constraint_reservation(const Constraint& constraint, ReservationTable& table) {
    switch (constraint.type) {
        case ConflictType::Vertex:
        case ConflictType::Follow:
            table.reserve_cell(constraint.cell.row, constraint.cell.col, constraint.time, -1);
            break;
        case ConflictType::Edge:
            // SpaceTimeAStar checks edge conflicts by querying reservations on
            // the reverse direction (next -> current). To forbid traversing
            // from A -> B at time t, we must therefore reserve B -> A at t.
            table.reserve_edge(
                constraint.to[0],
                constraint.from[0],
                constraint.time,
                -1
            );
            break;
        case ConflictType::None:
            break;
    }
}

} // namespace

Plan CBSSolver::solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) {
    const int num_agents = initial_state.num_agents();

    SpaceTimeAStar stastar(heuristic);
    AStar astar(heuristic);
    (void) astar;

    std::vector<AgentPlan> agent_plans(num_agents);
    AgentPlan plan;
    ReservationTable conflict_table;

    std::vector<CBSNode> CT;
    CT.reserve(2048);
    CBSNode start;

    CBSFrontier open(&CT);

    // initial plan for all agents individually
    for (int agent = 0; agent < num_agents; agent++) {
        plan = stastar.search(level, initial_state, agent, 20'000, conflict_table);
        if (!plan.valid()) {
            return Plan{};
        }
        agent_plans[agent] = plan;
    }

    start.plans = agent_plans;
    update_node_costs(start);
    CT.push_back(start);
    open.push(0);

    // go through the CT until a solution is found
    while (!open.empty()) {
        const int current_index = open.pop();
        const CBSNode current = CT[current_index];

        // detect conflicts
        const Conflict conflict = ConflictDetector::findFirstConflict(current.plans);
        if (conflict.agents[0] == -1) {
            return PlanMerger::merge_agent_plans(current.plans, num_agents);
        }

        for (int i = 0; i < 2; i++) {
            // create a new node
            CBSNode child;
            child.constraints = current.constraints;
            child.plans = current.plans;

            // add the new constraint
            Constraint constraint;
            constraint.agent_id = conflict.agents[i];
            constraint.time = conflict.time;
            constraint.type = conflict.type;
            constraint.cell = conflict.cell;
            constraint.from[0] = conflict.from[i];
            constraint.to[0] = conflict.to[i];
            child.constraints.push_back(constraint);

            // add the current constraints into the reservation table
            ReservationTable res_table;
            for (const Constraint& cons : child.constraints) {
                if (cons.agent_id != conflict.agents[i]) {
                    continue;
                }
                add_constraint_reservation(cons, res_table);
            }

            // search for the constrained agent
            plan = stastar.search(level, initial_state, conflict.agents[i], 20'000, res_table);
            if (plan == child.plans[conflict.agents[i]]) {
                std::cerr << "plan did not change!" << std::endl;
                continue;
            }
            if (!plan.valid()) {
                std::cerr << "plan is not valid!" << std::endl;
                continue;
            }
            
            child.plans[conflict.agents[i]] = plan;
            update_node_costs(child);

            // push the node into open
            CT.push_back(child);
            const int next_index = static_cast<int>(CT.size()) - 1;
            open.push(next_index);
        }
    }

    return Plan{};
}
