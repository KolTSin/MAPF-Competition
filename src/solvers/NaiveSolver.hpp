#include "solvers/Solver.hpp"
#include "actions/JointAction.hpp"
#include <memory>

class NaiveSolver final : public Solver {
public:
    Plan solve(const Level& level) override {
        Plan plan;
        JointAction ja;
        ja.actions.resize(level.agents.size(), Action{ActionType::NoOp, "NoOp"});
        plan.steps.push_back(ja);
        return plan;
    }
};

std::unique_ptr<Solver> make_naive_solver() {
    return std::make_unique<NaiveSolver>();
}