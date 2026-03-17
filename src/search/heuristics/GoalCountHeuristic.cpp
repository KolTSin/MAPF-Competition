#include "search/heuristics/GoalCountHeuristic.hpp"
#include "search/heuristics/HeuristicFactory.hpp"

GoalCountHeuristic::GoalCountHeuristic(const Level& level, const HeuristicContext& context)
    : level_(level), context_(context)
{
}

int GoalCountHeuristic::evaluate(const State& state) const
{
    int h = 0;

    for (char boxLetter = 'A'; boxLetter <= 'Z'; ++boxLetter)
    {
        const auto& goals = context_.boxGoalCells(boxLetter);
        for (int cellId : goals)
        {
            const int row = context_.rowOfCell(cellId);
            const int col = context_.colOfCell(cellId);

            if (context_.boxAt(state, row, col) != boxLetter)
                ++h;
        }
    }

    for (int agentId = 0; agentId < HeuristicContext::MAX_AGENT_IDS; ++agentId)
    {
        const auto& goals = context_.agentGoalCells(agentId);
        for (int cellId : goals)
        {
            const int row = context_.rowOfCell(cellId);
            const int col = context_.colOfCell(cellId);

            if (context_.agentAt(state, row, col) != agentId)
                ++h;
        }
    }

    return h;
}

REGISTER_HEURISTIC("gc", GoalCountHeuristic);