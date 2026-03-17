#include "ProgressHeuristic.hpp"
#include "HeuristicFactory.hpp"

ProgressHeuristic::ProgressHeuristic(const Level& level, const HeuristicContext& context)
    : level_(level), context_(context)
{
}

int ProgressHeuristic::evaluate(const State& state) const
{
    int goalCount = 0;
    int boxDistance = 0;
    int agentDistance = 0;

    for (char boxLetter = 'A'; boxLetter <= 'Z'; ++boxLetter)
    {
        const auto& goals = context_.boxGoalCells(boxLetter);
        for (int cellId : goals)
        {
            const int row = context_.rowOfCell(cellId);
            const int col = context_.colOfCell(cellId);

            if (context_.boxAt(state, row, col) != boxLetter)
                ++goalCount;
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
                ++goalCount;
        }
    }

    for (int row = 0; row < context_.rows(); ++row)
    {
        for (int col = 0; col < context_.cols(); ++col)
        {
            if (context_.isWall(row, col))
                continue;

            const char box = context_.boxAt(state, row, col);
            if (box < 'A' || box > 'Z')
                continue;

            const int id = context_.cellId(row, col);
            const int d = context_.distanceToNearestBoxGoal(box, id);

            if (d != HeuristicContext::INF)
                boxDistance += d;
        }
    }

    const int numAgents = context_.numAgents(state);
    for (int agentId = 0; agentId < numAgents; ++agentId)
    {
        const Position p = context_.agentPosition(state, agentId);
        const int id = context_.cellId(p);
        const int d = context_.distanceToAgentGoal(agentId, id);

        if (d != HeuristicContext::INF)
            agentDistance += d;
    }

    return goalCount + boxDistance + agentDistance;
}

REGISTER_HEURISTIC("progress", ProgressHeuristic);