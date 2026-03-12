#include "solvers/NaiveSolver.hpp"

#include "actions/Action.hpp"
#include "actions/JointAction.hpp"

Plan NaiveSolver::solve(const Level& level, const State& initial_state) {
    (void) level;

    Plan plan;
    JointAction ja;
    ja.actions.resize(initial_state.num_agents(), Action::noop());
    ja.actions.clear();
    // for (int i = 0; i < initial_state.num_agents(); ++i) {
    //     if (i % 4 == 0) {
    //         ja.actions.push_back(Action::move(Direction::North));
    //     } else if (i % 4 == 1) {
    //         ja.actions.push_back(Action::move(Direction::South));
    //     } else if (i % 4 == 2) {
    //         ja.actions.push_back(Action::move(Direction::East));
    //     } else {
    //         ja.actions.push_back(Action::noop());
    //     }
    // }
    ja.actions.push_back(Action::move(Direction::East));
    ja.actions.push_back(Action::move(Direction::West));
    plan.steps.push_back(ja);
    ja.actions.clear();
    ja.actions.push_back(Action::pull(Direction::West,Direction::West));
    ja.actions.push_back(Action::pull(Direction::East,Direction::East));
    plan.steps.push_back(ja);
    return plan;
}