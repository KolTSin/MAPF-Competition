#pragma once

#include "domain/Level.hpp"
#include "state/State.hpp"
#include "search/heuristics/Heuristic.hpp"
#include "search/heuristics/HeuristicContext.hpp"


class BoxGoalDistanceHeuristic : public IHeuristic {
public:
    BoxGoalDistanceHeuristic(const Level& level, const HeuristicContext& context);

    int evaluate(const State& state) const override;
    std::string name() const override { return "bgd"; }

private:
    const Level& level_;
    const HeuristicContext& context_;
};