#include "plan/PlanConflictRepairer.hpp"

#include "actions/ActionApplicator.hpp"
#include "actions/ActionSemantics.hpp"
#include "plan/AgentPlan.hpp"
#include "plan/ConflictDetector.hpp"
#include "plan/Conflicts.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

int num_agents_for(const State& state, const Plan& plan) {
    if (!state.agent_positions.empty()) {
        return state.num_agents();
    }

    if (!plan.steps.empty()) {
        return static_cast<int>(plan.steps.front().actions.size());
    }

    return 0;
}

Plan normalize_plan(const Plan& input, int num_agents) {
    Plan out = input;

    for (JointAction& step : out.steps) {
        if (static_cast<int>(step.actions.size()) < num_agents) {
            step.actions.resize(static_cast<std::size_t>(num_agents), Action::noop());
        } else if (static_cast<int>(step.actions.size()) > num_agents) {
            step.actions.resize(static_cast<std::size_t>(num_agents));
        }
    }

    return out;
}

std::vector<std::vector<Action>> timelines_from_plan(const Plan& plan, int num_agents) {
    std::vector<std::vector<Action>> timelines(static_cast<std::size_t>(num_agents));

    for (const JointAction& step : plan.steps) {
        for (int agent = 0; agent < num_agents; ++agent) {
            Action action = Action::noop();

            if (agent < static_cast<int>(step.actions.size())) {
                action = step.actions[static_cast<std::size_t>(agent)];
            }

            timelines[static_cast<std::size_t>(agent)].push_back(action);
        }
    }

    return timelines;
}

Plan plan_from_timelines(const std::vector<std::vector<Action>>& timelines) {
    Plan out;

    std::size_t makespan = 0;
    for (const auto& actions : timelines) {
        makespan = std::max(makespan, actions.size());
    }

    out.steps.resize(makespan);

    for (std::size_t t = 0; t < makespan; ++t) {
        out.steps[t].actions.resize(timelines.size(), Action::noop());

        for (std::size_t agent = 0; agent < timelines.size(); ++agent) {
            if (t < timelines[agent].size()) {
                out.steps[t].actions[agent] = timelines[agent][t];
            }
        }
    }

    return out;
}

bool is_executable(const Level& level, const State& initial_state, const Plan& plan) {
    State current = initial_state;

    for (const JointAction& step : plan.steps) {
        if (static_cast<int>(step.actions.size()) != current.num_agents()) {
            return false;
        }

        // Check applicability against the state at the beginning of the joint action.
        for (int agent = 0; agent < current.num_agents(); ++agent) {
            const Action& action = step.actions[static_cast<std::size_t>(agent)];

            if (!ActionApplicator::is_applicable(level, current, agent, action)) {
                return false;
            }
        }

        // Apply after all actions have been checked.
        for (int agent = 0; agent < current.num_agents(); ++agent) {
            const Action& action = step.actions[static_cast<std::size_t>(agent)];
            current = ActionApplicator::apply(level, current, agent, action);
        }
    }

    return true;
}

std::vector<Conflict> conflicts_for(const Plan& plan, const State& initial_state) {
    ConflictDetector detector;

    // Assumes you implemented the full detector from the previous step:
    // findAllConflicts(plan, initial_state, true)
    return detector.findAllConflicts(plan, initial_state, true);
}

bool is_conflict_free(const Plan& plan, const State& initial_state) {
    return conflicts_for(plan, initial_state).empty();
}

int first_conflict_time(const Plan& plan, const State& initial_state) {
    const std::vector<Conflict> conflicts = conflicts_for(plan, initial_state);

    if (conflicts.empty()) {
        return std::numeric_limits<int>::max();
    }

    return conflicts.front().time;
}

std::vector<int> affected_agents(const Conflict& conflict, int num_agents) {
    std::vector<int> agents;

    for (int a : conflict.agents) {
        if (a >= 0 && a < num_agents) {
            if (std::find(agents.begin(), agents.end(), a) == agents.end()) {
                agents.push_back(a);
            }
        }
    }

    return agents;
}

std::vector<int> candidate_insert_times(const Conflict& conflict, const Plan& plan) {
    std::vector<int> times;

    const int makespan = static_cast<int>(plan.steps.size());

    auto add = [&](int t) {
        t = std::max(0, std::min(t, makespan));

        if (std::find(times.begin(), times.end(), t) == times.end()) {
            times.push_back(t);
        }
    };

    // For vertex conflicts, conflict.time is usually t + 1,
    // so the causing action is at conflict.time - 1.
    add(conflict.time - 1);

    // For transition conflicts, conflict.time may already be the action index.
    add(conflict.time);

    // Sometimes delaying just after the conflict point helps when earlier delay
    // creates a new immediate conflict.
    add(conflict.time + 1);

    return times;
}

Plan insert_noop_for_agent(const Plan& plan, int num_agents, int agent, int insert_time) {
    auto timelines = timelines_from_plan(plan, num_agents);

    auto& actions = timelines[static_cast<std::size_t>(agent)];

    const int clamped_time =
        std::max(0, std::min(insert_time, static_cast<int>(actions.size())));

    const auto pos = actions.begin() + clamped_time;
    actions.insert(pos, Action::noop());

    return plan_from_timelines(timelines);
}

std::string plan_key(const Plan& plan) {
    std::ostringstream out;

    for (const JointAction& step : plan.steps) {
        for (const Action& action : step.actions) {
            out << action.to_string() << ';';
        }
        out << '|';
    }

    return out.str();
}

struct SearchNode {
    Plan plan;
    int inserted_noops{0};
    int first_conflict{std::numeric_limits<int>::max()};
    int plan_size{0};
    int id{0};
};

struct SearchNodeCompare {
    bool operator()(const SearchNode& a, const SearchNode& b) const {
        // Priority queue is max-heap by default, so "worse" should return true.

        if (a.first_conflict != b.first_conflict) {
            return a.first_conflict < b.first_conflict;
        }

        if (a.inserted_noops != b.inserted_noops) {
            return a.inserted_noops > b.inserted_noops;
        }

        if (a.plan_size != b.plan_size) {
            return a.plan_size > b.plan_size;
        }

        return a.id > b.id;
    }
};

} // namespace

PlanConflictRepairer::Result PlanConflictRepairer::repair(
    const Level& level,
    const State& initial_state,
    const Plan& input,
    int max_iterations,
    int max_extra_steps
) const {
    Result result;

    const int num_agents = num_agents_for(initial_state, input);
    const Plan root_plan = normalize_plan(input, num_agents);

    result.plan = root_plan;

    if (num_agents <= 1 || root_plan.steps.empty()) {
        result.conflict_free =
            is_conflict_free(root_plan, initial_state) &&
            is_executable(level, initial_state, root_plan);
        return result;
    }

    const std::size_t root_size = root_plan.steps.size();
    const std::size_t max_size =
        root_size + static_cast<std::size_t>(std::max(0, max_extra_steps));

    std::priority_queue<SearchNode, std::vector<SearchNode>, SearchNodeCompare> open;
    std::set<std::string> closed;

    int next_id = 0;

    SearchNode root;
    root.plan = root_plan;
    root.inserted_noops = 0;
    root.first_conflict = first_conflict_time(root_plan, initial_state);
    root.plan_size = static_cast<int>(root_plan.steps.size());
    root.id = next_id++;

    open.push(std::move(root));

    int expansions = 0;

    while (!open.empty() && expansions < max_iterations) {
        SearchNode node = std::move(const_cast<SearchNode&>(open.top()));
        open.pop();

        const std::string key = plan_key(node.plan);
        if (closed.find(key) != closed.end()) {
            continue;
        }
        closed.insert(key);

        result.plan = node.plan;
        result.iterations = expansions;

        const std::vector<Conflict> conflicts =
            conflicts_for(node.plan, initial_state);

        if (conflicts.empty()) {
            result.conflict_free = is_executable(level, initial_state, node.plan);
            result.changed = node.inserted_noops > 0;
            result.plan = node.plan;
            return result;
        }

        const Conflict& conflict = conflicts.front();

        const std::vector<int> agents = affected_agents(conflict, num_agents);

        // If the conflict detector returned a conflict with no responsible agent,
        // this repairer cannot branch meaningfully.
        if (agents.empty()) {
            result.conflict_free = false;
            result.changed = node.inserted_noops > 0;
            result.plan = node.plan;
            result.iterations = expansions;
            return result;
        }

        const std::vector<int> insert_times =
            candidate_insert_times(conflict, node.plan);

        for (int agent : agents) {
            for (int insert_time : insert_times) {
                Plan candidate =
                    insert_noop_for_agent(node.plan, num_agents, agent, insert_time);

                if (candidate.steps.size() > max_size) {
                    continue;
                }

                const std::string candidate_key = plan_key(candidate);
                if (closed.find(candidate_key) != closed.end()) {
                    continue;
                }

                // This rejects candidates where delayed actions become individually
                // inapplicable. Conflicts are handled separately by ConflictDetector.
                if (!is_executable(level, initial_state, candidate)) {
                    continue;
                }

                SearchNode child;
                child.plan = std::move(candidate);
                child.inserted_noops = node.inserted_noops + 1;
                child.first_conflict = first_conflict_time(child.plan, initial_state);
                child.plan_size = static_cast<int>(child.plan.steps.size());
                child.id = next_id++;

                open.push(std::move(child));
            }
        }

        ++expansions;
    }

    result.conflict_free =
        is_conflict_free(result.plan, initial_state) &&
        is_executable(level, initial_state, result.plan);

    result.changed = result.plan.steps.size() != root_plan.steps.size();
    result.iterations = expansions;

    return result;
}