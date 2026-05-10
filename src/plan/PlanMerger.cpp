#include "plan/PlanMerger.hpp"
#include "plan/AgentPlan.hpp"
#include "actions/JointAction.hpp"
#include "actions/ActionApplicator.hpp"
#include "plan/ConflictDetector.hpp"

#include <algorithm>

namespace {
bool is_global_noop(const JointAction& joint_action) {
    return std::all_of(joint_action.actions.begin(),
                       joint_action.actions.end(),
                       [](const Action& action) { return action.type == ActionType::NoOp; });
}

bool is_executable(const Level& level, const State& initial_state, const Plan& plan) {
    State current = initial_state;

    for (const JointAction& step : plan.steps) {
        if (static_cast<int>(step.actions.size()) != current.num_agents()) return false;

        for (int agent = 0; agent < current.num_agents(); ++agent) {
            const Action& action = step.actions[static_cast<std::size_t>(agent)];
            if (!ActionApplicator::is_applicable(level, current, agent, action)) return false;
        }

        for (int agent = 0; agent < current.num_agents(); ++agent) {
            current = ActionApplicator::apply(level, current, agent, step.actions[static_cast<std::size_t>(agent)]);
        }
    }

    return true;
}

bool is_valid_plan(const Level& level, const State& initial_state, const Plan& plan) {
    return is_executable(level, initial_state, plan) &&
           !ConflictDetector::has_conflict(plan, initial_state);
}
}

Plan PlanMerger::merge_agent_plans(const std::vector<AgentPlan>& agent_plans,
                                   int num_agents) {
    std::size_t horizon = 0;
    for (const auto& plan : agent_plans) {
        horizon = std::max(horizon, plan.actions.size());
    }

    Plan result;
    result.steps.reserve(horizon);

    for (std::size_t t = 0; t < horizon; ++t) {
        JointAction ja;
        ja.actions.resize(num_agents, Action::noop());

        for (int agent = 0; agent < num_agents; ++agent) {
            if (t < agent_plans[agent].actions.size()) {
                // std::cerr << agent_plans[agent][t].to_string() << '\n';
                ja.actions[agent] = agent_plans[agent].actions[t];
            }
        }
        // std::cerr << ja.to_string() << '\n';
        result.steps.push_back(std::move(ja));
    }

    compress_idle_steps(result);
    return result;
}

void PlanMerger::compress_idle_steps(Plan& plan) {
    plan.steps.erase(std::remove_if(plan.steps.begin(), plan.steps.end(), is_global_noop), plan.steps.end());
}


void PlanMerger::compact_independent_actions(const Level& level, const State& initial_state, Plan& plan) {
    compress_idle_steps(plan);
    if (!is_valid_plan(level, initial_state, plan)) return;

    bool changed = true;
    while (changed) {
        changed = false;

        for (std::size_t t = 1; t < plan.steps.size() && !changed; ++t) {
            const std::size_t agents = std::min(plan.steps[t - 1].actions.size(), plan.steps[t].actions.size());
            for (std::size_t agent = 0; agent < agents; ++agent) {
                if (plan.steps[t - 1].actions[agent].type != ActionType::NoOp) continue;
                if (plan.steps[t].actions[agent].type == ActionType::NoOp) continue;

                Plan candidate = plan;
                std::swap(candidate.steps[t - 1].actions[agent], candidate.steps[t].actions[agent]);
                compress_idle_steps(candidate);

                if (!is_valid_plan(level, initial_state, candidate)) continue;

                plan = std::move(candidate);
                changed = true;
                break;
            }
        }
    }
}
