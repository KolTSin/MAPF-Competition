#include "solvers/CBSSolver.hpp"

#include "plan/AgentPlan.hpp"
#include "plan/ConflictDetector.hpp"
#include "plan/PlanMerger.hpp"
#include "plan/ReservationTable.hpp"
#include "search/CBSFrontier.hpp"
#include "search/CBSNode.hpp"
#include "search/SpaceTimeAStar.hpp"
#include "search/heuristics/Heuristic.hpp"

#include <algorithm>
#include <vector>

namespace {

constexpr int kMaxLowLevelTime = 20'000;

bool is_known_position(Position p) {
    return p.row >= 0 && p.col >= 0;
}

void update_costs(CBSNode& node) {
    node.makespan = 0;
    node.sum_of_costs = 0;

    for (const AgentPlan& plan : node.plans) {
        const int cost = plan.cost();
        node.sum_of_costs += cost;
        node.makespan = std::max(node.makespan, cost);
    }
}

} // namespace

namespace cbs_solver_detail {

Constraint make_constraint_for_branch(const Conflict& conflict, int branch) {
    Constraint constraint;
    constraint.agent_id = conflict.agents[branch];
    constraint.time = conflict.time;
    constraint.type = conflict.type;
    constraint.cell = conflict.cell;
    constraint.from[0] = conflict.from[branch];
    constraint.to[0] = conflict.to[branch];
    return constraint;
}

void add_constraint_reservation(const Constraint& constraint, ReservationTable& reservations) {
    switch (constraint.type) {
        case ConflictType::AgentVertex:
            if (is_known_position(constraint.cell)) {
                reservations.reserve_cell(
                    constraint.cell.row,
                    constraint.cell.col,
                    constraint.time,
                    -1
                );
            }
            break;

        case ConflictType::AgentEdgeSwap:
            // SpaceTimeAStar rejects a transition from->to when the reverse
            // directed edge to->from is reserved at the same departure time.
            if (is_known_position(constraint.from[0]) && is_known_position(constraint.to[0])) {
                reservations.reserve_edge(constraint.to[0], constraint.from[0], constraint.time, -1);
            }
            break;

        case ConflictType::AgentFollow:
            // Follow conflicts are transition conflicts: constrain only the
            // selected branch agent's move that produced the follow relation.
            if (is_known_position(constraint.from[0]) && is_known_position(constraint.to[0])) {
                reservations.reserve_edge(constraint.to[0], constraint.from[0], constraint.time, -1);
            } else if (is_known_position(constraint.cell)) {
                reservations.reserve_cell(
                    constraint.cell.row,
                    constraint.cell.col,
                    constraint.time,
                    -1
                );
            }
            break;

        default:
            break;
    }
}

ReservationTable build_reservations_for_agent(const std::vector<Constraint>& constraints, int agent) {
    ReservationTable reservations;
    for (const Constraint& constraint : constraints) {
        if (constraint.agent_id == agent) {
            add_constraint_reservation(constraint, reservations);
        }
    }
    return reservations;
}

} // namespace cbs_solver_detail

namespace {

bool all_plans_valid(const std::vector<AgentPlan>& plans) {
    return std::all_of(plans.begin(), plans.end(), [](const AgentPlan& plan) {
        return plan.valid();
    });
}

} // namespace

Plan CBSSolver::solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) {
    const int num_agents = initial_state.num_agents();

    SpaceTimeAStar low_level(heuristic);

    std::vector<AgentPlan> agent_plans(num_agents);
    ReservationTable empty_reservations;

    for (int agent = 0; agent < num_agents; ++agent) {
        agent_plans[agent] = low_level.search(level, initial_state, agent, kMaxLowLevelTime, empty_reservations);
    }

    if (!all_plans_valid(agent_plans)) {
        return Plan{};
    }

    std::vector<CBSNode> constraint_tree;
    constraint_tree.reserve(2048);

    CBSNode root;
    root.plans = std::move(agent_plans);
    update_costs(root);

    constraint_tree.push_back(std::move(root));

    CBSFrontier open(&constraint_tree);
    open.push(0);

    ConflictDetector conflict_detector;

    while (!open.empty()) {
        const int current_index = open.pop();
        const CBSNode& current = constraint_tree[current_index];

        const Conflict conflict = conflict_detector.findFirstConflict(current.plans);
        if (!conflict.valid()) {
            return PlanMerger::merge_agent_plans(current.plans, num_agents);
        }

        const std::vector<Constraint> current_constraints = current.constraints;
        const std::vector<AgentPlan> current_plans = current.plans;

        for (int branch = 0; branch < 2; ++branch) {
            const int constrained_agent = conflict.agents[branch];
            if (constrained_agent < 0 || constrained_agent >= num_agents) {
                continue;
            }

            CBSNode child;
            child.constraints = current_constraints;
            child.plans = current_plans;
            child.constraints.push_back(cbs_solver_detail::make_constraint_for_branch(conflict, branch));

            const ReservationTable reservations = cbs_solver_detail::build_reservations_for_agent(
                child.constraints,
                constrained_agent
            );

            AgentPlan repaired_plan = low_level.search(
                level,
                initial_state,
                constrained_agent,
                kMaxLowLevelTime,
                reservations
            );

            if (!repaired_plan.valid() || repaired_plan == child.plans[constrained_agent]) {
                continue;
            }

            child.plans[constrained_agent] = std::move(repaired_plan);
            update_costs(child);

            constraint_tree.push_back(std::move(child));
            open.push(static_cast<int>(constraint_tree.size()) - 1);
        }
    }

    return Plan{};
}
