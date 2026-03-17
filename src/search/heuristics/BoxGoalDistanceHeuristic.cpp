#include "BoxGoalDistanceHeuristic.hpp"
#include "HeuristicFactory.hpp"

BoxGoalDistanceHeuristic::BoxGoalDistanceHeuristic(
    const Level& level,
    const HeuristicContext& context
)
    : level_(level), context_(context)
{
}

int BoxGoalDistanceHeuristic::evaluate(const State& state) const
{
    int h = 0;

    for (int row = 0; row < context_.rows(); ++row)
    {
        for (int col = 0; col < context_.cols(); ++col)
        {
            if (context_.isWall(row, col))
                continue;

            const char box = context_.boxAt(state, row, col);
            if (box < 'A' || box > 'Z')
                continue;

            const int cellId = context_.cellId(row, col);
            const int d = context_.distanceToNearestBoxGoal(box, cellId);

            if (d != HeuristicContext::INF)
                h += d;
        }
    }

    return h;
}

REGISTER_HEURISTIC("bgd", BoxGoalDistanceHeuristic);