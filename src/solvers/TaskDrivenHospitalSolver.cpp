#include "solvers/TaskDrivenHospitalSolver.hpp"

#include "actions/Action.hpp"
#include "actions/JointAction.hpp"
#include "hospital/BoxTransportPlanner.hpp"
#include "hospital/HospitalTaskDecomposer.hpp"
#include "hospital/MapAnalysis.hpp"

#include <iostream>

namespace {
void append_single_agent_actions(
    Plan& plan,
    const std::vector<Action>& actions,
    int num_agents,
    int agent_id) {
    for (const Action& action : actions) {
        JointAction ja;
        ja.actions.assign(static_cast<std::size_t>(num_agents), Action::noop());
        ja.actions[agent_id] = action;
        plan.steps.push_back(std::move(ja));
    }
}
}

Plan TaskDrivenHospitalSolver::solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) {
    (void) heuristic;

    Plan plan;
    State current = initial_state;

    MapAnalysis analysis(level);
    HospitalTaskDecomposer decomposer;
    BoxTransportPlanner planner(analysis);

    const std::vector<HospitalTask> tasks = decomposer.decompose(level, current, analysis);

    std::cerr << "TaskDrivenHospitalSolver generated tasks=" << tasks.size() << '\n';

    for (const HospitalTask& task : tasks) {
        if (task.type == HospitalTaskType::AssignBox) {
            std::cerr << "[assign] agent=" << task.agent_id
                      << " box=" << task.box_symbol
                      << " -> goal=(" << task.destination.row << "," << task.destination.col << ")"
                      << " reason=" << task.reason
                      << '\n';
            continue;
        }

        const std::vector<Action> segment = planner.plan_for_task(level, current, task);
        append_single_agent_actions(plan, segment, current.num_agents(), task.agent_id);

        std::cerr << "[execute] type="
                  << (task.type == HospitalTaskType::RelocateBox ? "relocate" : "transport")
                  << " agent=" << task.agent_id
                  << " box=" << task.box_symbol
                  << " actions=" << segment.size()
                  << " reason=" << task.reason
                  << '\n';
    }

    return plan;
}
