#pragma once

#include "domain/Level.hpp"
#include "state/State.hpp"
#include "search/heuristics/Heuristic.hpp"
#include "search/heuristics/HeuristicContext.hpp"


// Sums static distances from each agent to its digit goal.
class AgentGoalDistanceHeuristic : public IHeuristic {
public:
    AgentGoalDistanceHeuristic(const Level& level, const HeuristicContext& context);

    int evaluate(const State& state) const override;
    std::string name() const override { return "agd"; }

private:
    const Level& level_;
    const HeuristicContext& context_;
};