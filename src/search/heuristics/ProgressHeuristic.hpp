#pragma once

#include "Heuristic.hpp"
#include "HeuristicContext.hpp"

#include "../../domain/Level.hpp"

class ProgressHeuristic : public IHeuristic
{
public:
    ProgressHeuristic(const Level& level, const HeuristicContext& context);

    int evaluate(const State& state) const override;
    std::string name() const override { return "progress"; }

private:
    const Level& level_;
    const HeuristicContext& context_;
};