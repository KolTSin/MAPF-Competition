#include "search/SuccessorGenerator.hpp"

#include "actions/ActionLibrary.hpp"
#include "actions/ActionApplicator.hpp"

void SuccessorGenerator::expand_agent(
    const Level& level,
    const State& state,
    int agent_id,
    std::vector<Successor>& out)
{
    out.clear();

    for (const Action& action : ActionLibrary::ALL_ACTIONS)
    {
        if (ActionApplicator::is_applicable(level, state, agent_id, action))
        {
            State next = ActionApplicator::apply(level, state, agent_id, action);

            out.push_back(Successor{
                action,
                std::move(next)
            });
        }
    }
}