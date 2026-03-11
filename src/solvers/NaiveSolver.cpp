#include "solvers/NaiveSolver.hpp"

Plan NaiveSolver::solve(const Level& level) {
    Plan plan;
    JointAction ja;
    ja.actions.resize(level.agents.size(), Action{ActionType::NoOp, "NoOp"});
    plan.steps.push_back(ja);
    return plan;
}