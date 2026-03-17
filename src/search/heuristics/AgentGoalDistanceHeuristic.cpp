#include "search/heuristics/AgentGoalDistanceHeuristic.hpp"
#include "search/heuristics/HeuristicFactory.hpp"

AgentGoalDistanceHeuristic::AgentGoalDistanceHeuristic(
    const Level& level,
    const HeuristicContext& context
)
    : level_(level), context_(context)
{
}

int AgentGoalDistanceHeuristic::evaluate(const State& state) const
{
    int h = 0;
    const int numAgents = context_.numAgents(state);

    for (int agentId = 0; agentId < numAgents; ++agentId)
    {
        const Position pos = context_.agentPosition(state, agentId);
        const int cellId = context_.cellId(pos);
        const int d = context_.distanceToAgentGoal(agentId, cellId);

        if (d != HeuristicContext::INF)
            h += d;
    }

    return h;
}

REGISTER_HEURISTIC("agd", AgentGoalDistanceHeuristic);