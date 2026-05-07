#pragma once

#include "domain/Level.hpp"
#include "state/State.hpp"
#include "search/heuristics/Heuristic.hpp"
#include "search/heuristics/HeuristicContext.hpp"


// Counts unsatisfied goals; cheap but coarse progress estimate.
class GoalCountHeuristic : public IHeuristic {
public:
    GoalCountHeuristic(const Level& level, const HeuristicContext& context);

    int evaluate(const State& state) const override;
    std::string name() const override { return "gc"; }

private:
    const Level& level_;
    const HeuristicContext& context_;
};