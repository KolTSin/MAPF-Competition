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
#include <sstream>
#include <string>
#include <unordered_set>

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

std::string encode_constraint(const Constraint& constraint) {
    std::ostringstream os;
    os << constraint.agent_id
       << '|'
       << static_cast<int>(constraint.type)
       << '|'
       << constraint.time
       << '|'
       << constraint.cell.row
       << ','
       << constraint.cell.col
       << '|'
       << constraint.from[0].row
       << ','
       << constraint.from[0].col
       << '|'
       << constraint.to[0].row
       << ','
       << constraint.to[0].col;
    return os.str();
}

std::string build_constraint_signature(const std::vector<Constraint>& constraints) {
    std::vector<std::string> encoded;
    encoded.reserve(constraints.size());
    for (const auto& constraint : constraints) {
        encoded.push_back(encode_constraint(constraint));
    }
    std::sort(encoded.begin(), encoded.end());

    std::ostringstream signature;
    for (const auto& token : encoded) {
        signature << token << ';';
    }
    return signature.str();
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
    std::unordered_set<std::string> visited_constraint_sets;
    visited_constraint_sets.reserve(4096);

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
    visited_constraint_sets.insert(build_constraint_signature(start.constraints));

    // go through the CT until a solution is found
    while (!open.empty()) {
        const int current_index = open.pop();
        const CBSNode& current = CT[current_index];

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
            const std::string child_signature = build_constraint_signature(child.constraints);
            if (visited_constraint_sets.find(child_signature) != visited_constraint_sets.end()) {
                continue;
            }

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
            if (!plan.valid()) {
                continue;
            }
            if (plan == child.plans[conflict.agents[i]]) {
                continue;
            }
            child.plans[conflict.agents[i]] = plan;
            update_node_costs(child);

            // push the node into open
            CT.push_back(child);
            const int next_index = static_cast<int>(CT.size()) - 1;
            open.push(next_index);
            visited_constraint_sets.insert(child_signature);
        }
    }

    return Plan{};
}
