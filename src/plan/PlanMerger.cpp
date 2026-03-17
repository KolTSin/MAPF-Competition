#include "plan/PlanMerger.hpp"
#include "actions/JointAction.hpp"

#include <algorithm>
#include <iostream>

Plan PlanMerger::merge_agent_plans(const std::vector<std::vector<Action>>& agent_plans,
                                   int num_agents) {
    std::size_t horizon = 0;
    for (const auto& plan : agent_plans) {
        horizon = std::max(horizon, plan.size());
    }

    Plan result;
    result.steps.reserve(horizon);
    std::cerr << horizon << '\n';

    for (std::size_t t = 0; t < horizon; ++t) {
        JointAction ja;
        ja.actions.resize(num_agents, Action::noop());

        for (int agent = 0; agent < num_agents; ++agent) {
            if (t < agent_plans[agent].size()) {
                std::cerr << agent_plans[agent][t].to_string() << '\n';
                ja.actions[agent] = agent_plans[agent][t];
            }
        }
        std::cerr << ja.to_string() << '\n';
        result.steps.push_back(std::move(ja));
    }

    return result;
}