#include "actions/Action.hpp"
#include "actions/ActionApplicator.hpp"
#include "actions/ActionSemantics.hpp"
#include "parser/LevelParser.hpp"
#include "plan/AgentPlan.hpp"
#include "plan/ConflictDetector.hpp"
#include "plan/Plan.hpp"
#include "plan/ReservationTable.hpp"
#include "solvers/CBSSolver.hpp"
#include "search/heuristics/Heuristic.hpp"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

class ConstantHeuristic : public IHeuristic {
public:
    int evaluate(const State&) const override { return 0; }
    std::string name() const override { return "constant"; }
};

static AgentPlan make_agent_plan(int agent, std::vector<Position> positions) {
    AgentPlan plan;
    plan.agent = agent;
    plan.positions = std::move(positions);
    if (!plan.positions.empty()) {
        plan.actions.assign(plan.positions.size() - 1, Action::noop());
    }
    return plan;
}

static std::string resolve_test_path(const std::string& path) {
    if (std::filesystem::exists(path)) {
        return path;
    }
    return "../" + path;
}

static bool all_goals_satisfied(const Level& level, const State& state, std::string* reason) {
    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const char goal = level.goal_at(r, c);
            if (goal >= '0' && goal <= '9') {
                const int agent_id = goal - '0';
                if (agent_id >= state.num_agents() || !(state.agent_positions[agent_id] == Position{r, c})) {
                    if (reason != nullptr) {
                        std::ostringstream out;
                        out << "agent_goal_unsatisfied " << goal << " at (" << r << "," << c << ")";
                        *reason = out.str();
                    }
                    return false;
                }
            }
        }
    }
    return true;
}

static bool plan_solves_without_conflicts(const Level& level,
                                          const State& initial_state,
                                          const Plan& plan,
                                          std::string* reason) {
    if (plan.empty()) {
        if (reason != nullptr) *reason = "empty_plan";
        return false;
    }

    std::string conflict_reason;
    if (ConflictDetector::has_conflict(plan, initial_state, &conflict_reason)) {
        if (reason != nullptr) *reason = "conflict: " + conflict_reason;
        return false;
    }

    State current = initial_state;
    for (std::size_t t = 0; t < plan.steps.size(); ++t) {
        const JointAction& step = plan.steps[t];
        if (step.actions.size() != static_cast<std::size_t>(initial_state.num_agents())) {
            if (reason != nullptr) *reason = "wrong_joint_action_arity";
            return false;
        }

        std::vector<ActionEffect> effects;
        effects.reserve(step.actions.size());
        for (int agent_id = 0; agent_id < current.num_agents(); ++agent_id) {
            const Action& action = step.actions[static_cast<std::size_t>(agent_id)];
            if (!ActionApplicator::is_applicable(level, current, agent_id, action)) {
                if (reason != nullptr) {
                    std::ostringstream out;
                    out << "inapplicable_action at t=" << t << " agent=" << agent_id
                        << " action=" << action.to_string();
                    *reason = out.str();
                }
                return false;
            }
            effects.push_back(ActionSemantics::compute_effect(current.agent_positions[agent_id], action));
        }

        State next = current;
        for (int agent_id = 0; agent_id < current.num_agents(); ++agent_id) {
            next.agent_positions[agent_id] = effects[static_cast<std::size_t>(agent_id)].agent_to;
        }
        current = std::move(next);
    }

    return all_goals_satisfied(level, current, reason);
}

int main() {
    {
        Conflict vertex = ConflictDetector::findFirstConflict({
            make_agent_plan(0, {Position{1,1}, Position{1,2}}),
            make_agent_plan(1, {Position{1,3}, Position{1,2}})
        });
        assert(vertex.valid());
        assert(vertex.type == ConflictType::AgentVertex);
        assert(vertex.agents[0] == 0);
        assert(vertex.agents[1] == 1);
        assert(vertex.time == 1);
        assert((vertex.cell == Position{1,2}));

        Constraint vertex_c0 = cbs_solver_detail::make_constraint_for_branch(vertex, 0);
        Constraint vertex_c1 = cbs_solver_detail::make_constraint_for_branch(vertex, 1);
        assert(vertex_c0.agent_id == 0);
        assert(vertex_c1.agent_id == 1);
        assert(vertex_c0.type == ConflictType::AgentVertex);
        assert(vertex_c0.time == 1);
        assert((vertex_c0.cell == Position{1,2}));

        ReservationTable vertex_r0 = cbs_solver_detail::build_reservations_for_agent({vertex_c0, vertex_c1}, 0);
        ReservationTable vertex_r1 = cbs_solver_detail::build_reservations_for_agent({vertex_c0, vertex_c1}, 1);
        assert(vertex_r0.is_cell_reserved(1, 2, 1, 0));
        assert(vertex_r1.is_cell_reserved(1, 2, 1, 1));
    }

    {
        Conflict edge_swap = ConflictDetector::findFirstConflict({
            make_agent_plan(0, {Position{1,1}, Position{1,2}}),
            make_agent_plan(1, {Position{1,2}, Position{1,1}})
        });
        assert(edge_swap.valid());
        assert(edge_swap.type == ConflictType::AgentEdgeSwap);
        assert(edge_swap.time == 0);
        assert((edge_swap.from[0] == Position{1,1}));
        assert((edge_swap.to[0] == Position{1,2}));
        assert((edge_swap.from[1] == Position{1,2}));
        assert((edge_swap.to[1] == Position{1,1}));

        Constraint edge_c0 = cbs_solver_detail::make_constraint_for_branch(edge_swap, 0);
        Constraint edge_c1 = cbs_solver_detail::make_constraint_for_branch(edge_swap, 1);
        ReservationTable edge_r0 = cbs_solver_detail::build_reservations_for_agent({edge_c0, edge_c1}, 0);
        ReservationTable edge_r1 = cbs_solver_detail::build_reservations_for_agent({edge_c0, edge_c1}, 1);
        assert(edge_r0.is_edge_reserved(Position{1,2}, Position{1,1}, 0, 0));
        assert(edge_r1.is_edge_reserved(Position{1,1}, Position{1,2}, 0, 1));
    }

    {
        Conflict follow = ConflictDetector::findFirstConflict({
            make_agent_plan(0, {Position{1,1}, Position{1,2}}),
            make_agent_plan(1, {Position{1,2}, Position{1,3}})
        });
        assert(follow.valid());
        assert(follow.type == ConflictType::AgentFollow);
        assert(follow.agents[0] == 0);
        assert(follow.agents[1] == 1);
        assert(follow.time == 0);
        assert((follow.cell == Position{1,2}));
        assert((follow.from[0] == Position{1,1}));
        assert((follow.to[0] == Position{1,2}));

        Constraint follow_c0 = cbs_solver_detail::make_constraint_for_branch(follow, 0);
        ReservationTable follow_r0 = cbs_solver_detail::build_reservations_for_agent({follow_c0}, 0);
        assert(follow_r0.is_edge_reserved(Position{1,2}, Position{1,1}, 0, 0));
    }

    {
        const std::string resolved = resolve_test_path("testlevels/CBS_vertex_crossing.lvl");
        assert(std::filesystem::exists(resolved));
        std::ifstream in(resolved);
        assert(in.good());
        ParsedLevel parsed = LevelParser::parse(in);

        CBSSolver solver;
        ConstantHeuristic h;
        Plan plan = solver.solve(parsed.level, parsed.initial_state, h);
        std::string reason;
        if (!plan_solves_without_conflicts(parsed.level, parsed.initial_state, plan, &reason)) {
            std::cerr << "CBS vertex crossing failure: " << reason << "\n";
        }
        assert(plan_solves_without_conflicts(parsed.level, parsed.initial_state, plan, nullptr));
    }

    std::cout << "cbs_solver_tests passed\n";
    return 0;
}
