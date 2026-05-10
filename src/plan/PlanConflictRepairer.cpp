#include "plan/PlanConflictRepairer.hpp"

#include "actions/ActionApplicator.hpp"
#include "actions/ActionLibrary.hpp"
#include "actions/ActionSemantics.hpp"
#include "plan/AgentPlan.hpp"
#include "plan/ConflictDetector.hpp"
#include "plan/Conflicts.hpp"
#include "plan/PlanMerger.hpp"
#include "plan/ReservationTable.hpp"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
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

Plan plan_from_agent_plans(const std::vector<AgentPlan>& plans, int num_agents) {
    return PlanMerger::merge_agent_plans(plans, num_agents);
}

Plan uncompressed_plan_from_agent_plans(const std::vector<AgentPlan>& plans, int num_agents) {
    std::size_t horizon = 0;
    for (const AgentPlan& plan : plans) {
        horizon = std::max(horizon, plan.actions.size());
    }

    Plan out;
    out.steps.resize(horizon);
    for (std::size_t t = 0; t < horizon; ++t) {
        out.steps[t].actions.resize(static_cast<std::size_t>(num_agents), Action::noop());
        for (int agent = 0; agent < num_agents && agent < static_cast<int>(plans.size()); ++agent) {
            const AgentPlan& plan = plans[static_cast<std::size_t>(agent)];
            if (t < plan.actions.size()) {
                out.steps[t].actions[static_cast<std::size_t>(agent)] = plan.actions[t];
            }
        }
    }

    return out;
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

std::vector<Conflict> conflicts_for(const std::vector<AgentPlan>& plans, const State& initial_state) {
    ConflictDetector detector;
    return detector.findAllConflicts(plans, initial_state, true);
}

bool is_executable(const Level& level,
                   const State& initial_state,
                   const std::vector<AgentPlan>& plans) {
    const Plan plan = uncompressed_plan_from_agent_plans(plans, initial_state.num_agents());
    return is_executable(level, initial_state, plan);
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

bool is_conflict_free(const std::vector<AgentPlan>& plans, const State& initial_state) {
    return conflicts_for(plans, initial_state).empty();
}

int first_conflict_time(const Plan& plan, const State& initial_state) {
    const std::vector<Conflict> conflicts = conflicts_for(plan, initial_state);

    if (conflicts.empty()) {
        return std::numeric_limits<int>::max();
    }

    return conflicts.front().time;
}

struct ReplanTarget {
    Position agent{-1, -1};
    std::vector<char> boxes;
};

ReplanTarget target_from_agent_plan(const State& initial_state, const AgentPlan& plan) {
    ReplanTarget target;
    target.agent = plan.positions.empty() ? Position{-1, -1} : plan.positions.back();
    target.boxes = initial_state.box_pos;

    for (std::size_t i = 0; i < plan.actions.size(); ++i) {
        if (i >= plan.positions.size()) {
            break;
        }

        const Action& action = plan.actions[i];
        if (action.type != ActionType::Push && action.type != ActionType::Pull) {
            continue;
        }

        const ActionEffect effect = ActionSemantics::compute_effect(plan.positions[i], action);
        if (!initial_state.in_bounds(effect.box_from.row, effect.box_from.col) ||
            !initial_state.in_bounds(effect.box_to.row, effect.box_to.col)) {
            continue;
        }

        const int from_index = initial_state.index(effect.box_from.row, effect.box_from.col);
        const int to_index = initial_state.index(effect.box_to.row, effect.box_to.col);
        const char moved = target.boxes[static_cast<std::size_t>(from_index)];
        if (moved == '\0') {
            continue;
        }

        target.boxes[static_cast<std::size_t>(from_index)] = '\0';
        target.boxes[static_cast<std::size_t>(to_index)] = moved;
    }

    return target;
}

bool matches_replan_target(const State& state, int agent, const ReplanTarget& target) {
    if (agent < 0 || agent >= state.num_agents()) {
        return false;
    }
    return state.agent_positions[static_cast<std::size_t>(agent)] == target.agent &&
           state.box_pos == target.boxes;
}

struct ReplanKey {
    Position agent;
    std::vector<char> boxes;
    int time{0};

    bool operator==(const ReplanKey& other) const noexcept {
        return agent == other.agent && time == other.time && boxes == other.boxes;
    }
};

struct ReplanKeyHasher {
    std::size_t operator()(const ReplanKey& key) const noexcept {
        std::size_t seed = std::hash<int>{}(key.agent.row);
        auto combine = [&seed](std::size_t value) {
            seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        };
        combine(std::hash<int>{}(key.agent.col));
        combine(std::hash<int>{}(key.time));
        for (char box : key.boxes) {
            combine(std::hash<int>{}(static_cast<int>(box)));
        }
        return seed;
    }
};

struct ReplanNode {
    State state;
    int time{0};
    int g{0};
    int h{0};
    int parent{-1};
    Action action{Action::noop()};

    int f() const noexcept { return g + h; }
};

int replan_heuristic(const State& state, int agent, const ReplanTarget& target) {
    int h = 0;
    if (agent >= 0 && agent < state.num_agents()) {
        const Position pos = state.agent_positions[static_cast<std::size_t>(agent)];
        h += std::abs(pos.row - target.agent.row) + std::abs(pos.col - target.agent.col);
    }

    for (int i = 0; i < static_cast<int>(state.box_pos.size()) && i < static_cast<int>(target.boxes.size()); ++i) {
        const char box = state.box_pos[static_cast<std::size_t>(i)];
        if (box == '\0') {
            continue;
        }
        if (target.boxes[static_cast<std::size_t>(i)] == box) {
            continue;
        }

        Position from{i / state.cols, i % state.cols};
        int best = state.rows + state.cols;
        for (int j = 0; j < static_cast<int>(target.boxes.size()); ++j) {
            if (target.boxes[static_cast<std::size_t>(j)] != box ||
                (j < static_cast<int>(state.box_pos.size()) && state.box_pos[static_cast<std::size_t>(j)] == box)) {
                continue;
            }
            Position to{j / state.cols, j % state.cols};
            best = std::min(best, std::abs(from.row - to.row) + std::abs(from.col - to.col));
        }
        h += best;
    }

    return h;
}

std::optional<AgentPlan> reconstruct_replanned_agent(const std::vector<ReplanNode>& nodes,
                                                     int goal_index,
                                                     int agent) {
    AgentPlan plan;
    plan.agent = agent;

    int current = goal_index;
    while (current >= 0) {
        plan.positions.push_back(nodes[static_cast<std::size_t>(current)].state.agent_positions[static_cast<std::size_t>(agent)]);
        if (nodes[static_cast<std::size_t>(current)].parent >= 0) {
            plan.actions.push_back(nodes[static_cast<std::size_t>(current)].action);
        }
        current = nodes[static_cast<std::size_t>(current)].parent;
    }

    std::reverse(plan.positions.begin(), plan.positions.end());
    std::reverse(plan.actions.begin(), plan.actions.end());
    if (!plan.valid()) {
        return std::nullopt;
    }
    return plan;
}

ReservationTable reservations_for_other_plans(const std::vector<AgentPlan>& plans, int replanned_agent) {
    ReservationTable reservations;
    for (int agent = 0; agent < static_cast<int>(plans.size()); ++agent) {
        if (agent == replanned_agent) {
            continue;
        }
        if (plans[static_cast<std::size_t>(agent)].valid()) {
            reservations.reserve_path(plans[static_cast<std::size_t>(agent)]);
        }
    }
    return reservations;
}

bool violates_conflict_constraint(const Conflict& conflict,
                                  int agent,
                                  const ActionEffect& effect,
                                  int time) {
    const bool conflict_mentions_agent =
        conflict.agents[0] == agent || conflict.agents[1] == agent;
    if (!conflict_mentions_agent) {
        return false;
    }

    if (time + 1 == conflict.time && effect.agent_to == conflict.cell) {
        return true;
    }
    if (time == conflict.time && effect.agent_to == conflict.cell) {
        return true;
    }
    return false;
}

std::optional<AgentPlan> replan_agent_with_constraints(const Level& level,
                                                       const State& initial_state,
                                                       const std::vector<AgentPlan>& plans,
                                                       int agent,
                                                       const Conflict& conflict,
                                                       int max_extra_steps) {
    if (agent < 0 || agent >= static_cast<int>(plans.size()) || !plans[static_cast<std::size_t>(agent)].valid()) {
        return std::nullopt;
    }

    const AgentPlan& original = plans[static_cast<std::size_t>(agent)];
    const ReplanTarget target = target_from_agent_plan(initial_state, original);
    if (target.agent.row < 0 || target.agent.col < 0) {
        return std::nullopt;
    }

    const int max_time = static_cast<int>(original.actions.size()) + std::max(8, max_extra_steps);
    const int max_expansions = std::max(1000, level.rows * level.cols * std::max(50, max_time) * 20);
    const ReservationTable reservations = reservations_for_other_plans(plans, agent);

    std::vector<ReplanNode> nodes;
    nodes.reserve(2048);
    struct QueueCompare {
        const std::vector<ReplanNode>* nodes{nullptr};
        bool operator()(int a, int b) const {
            const ReplanNode& lhs = (*nodes)[static_cast<std::size_t>(a)];
            const ReplanNode& rhs = (*nodes)[static_cast<std::size_t>(b)];
            if (lhs.f() != rhs.f()) return lhs.f() > rhs.f();
            if (lhs.g != rhs.g) return lhs.g < rhs.g;
            return a > b;
        }
    };
    std::priority_queue<int, std::vector<int>, QueueCompare> open{QueueCompare{&nodes}};
    std::unordered_map<ReplanKey, int, ReplanKeyHasher> best_g;

    ReplanNode start;
    start.state = initial_state;
    start.h = replan_heuristic(initial_state, agent, target);
    nodes.push_back(start);
    open.push(0);
    best_g[ReplanKey{initial_state.agent_positions[static_cast<std::size_t>(agent)], initial_state.box_pos, 0}] = 0;

    int expansions = 0;
    while (!open.empty() && expansions < max_expansions) {
        const int current_index = open.top();
        open.pop();
        const ReplanNode current = nodes[static_cast<std::size_t>(current_index)];
        const ReplanKey current_key{
            current.state.agent_positions[static_cast<std::size_t>(agent)],
            current.state.box_pos,
            current.time
        };
        const auto best_it = best_g.find(current_key);
        if (best_it != best_g.end() && current.g > best_it->second) {
            continue;
        }

        if (matches_replan_target(current.state, agent, target)) {
            return reconstruct_replanned_agent(nodes, current_index, agent);
        }
        if (current.time >= max_time) {
            continue;
        }

        for (const Action& action : ActionLibrary::ALL_ACTIONS) {
            if (!ActionApplicator::is_applicable(level, current.state, agent, action)) {
                continue;
            }

            const Position agent_from = current.state.agent_positions[static_cast<std::size_t>(agent)];
            const ActionEffect effect = ActionSemantics::compute_effect(agent_from, action);
            if (violates_conflict_constraint(conflict, agent, effect, current.time)) {
                continue;
            }

            std::optional<char> moved_box;
            std::optional<Position> box_from;
            std::optional<Position> box_to;
            if (effect.moves_box && current.state.in_bounds(effect.box_from.row, effect.box_from.col)) {
                const char box = current.state.box_at(effect.box_from.row, effect.box_from.col);
                if (box != '\0') {
                    moved_box = box;
                    box_from = effect.box_from;
                    box_to = effect.box_to;
                }
            }

            if (!reservations.can_apply_transition(agent,
                                                   agent_from,
                                                   effect.agent_to,
                                                   moved_box,
                                                   box_from,
                                                   box_to,
                                                   current.time)) {
                continue;
            }

            ReplanNode next;
            next.state = ActionApplicator::apply(level, current.state, agent, action);
            next.time = current.time + 1;
            next.g = current.g + 1;
            next.h = replan_heuristic(next.state, agent, target);
            next.parent = current_index;
            next.action = action;

            const ReplanKey key{
                next.state.agent_positions[static_cast<std::size_t>(agent)],
                next.state.box_pos,
                next.time
            };
            const auto it = best_g.find(key);
            if (it != best_g.end() && it->second <= next.g) {
                continue;
            }
            best_g[key] = next.g;
            nodes.push_back(std::move(next));
            open.push(static_cast<int>(nodes.size()) - 1);
        }
        ++expansions;
    }

    return std::nullopt;
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
    // so the causing action is at conflict.time - 1. Also consider one
    // step earlier so a stationary blocker can move away before follow
    // conflicts are introduced.
    add(conflict.time - 2);
    add(conflict.time - 1);

    // For transition conflicts, conflict.time may already be the action index.
    add(conflict.time);

    // Sometimes delaying just after the conflict point helps when earlier delay
    // creates a new immediate conflict.
    add(conflict.time + 1);

    return times;
}

Plan insert_actions_for_agent(const Plan& plan,
                              int num_agents,
                              int agent,
                              int insert_time,
                              const std::vector<Action>& inserted_actions) {
    auto timelines = timelines_from_plan(plan, num_agents);

    auto& actions = timelines[static_cast<std::size_t>(agent)];

    const int clamped_time =
        std::max(0, std::min(insert_time, static_cast<int>(actions.size())));

    const auto pos = actions.begin() + clamped_time;
    actions.insert(pos, inserted_actions.begin(), inserted_actions.end());

    return plan_from_timelines(timelines);
}

Plan insert_noops_for_agent(const Plan& plan,
                            int num_agents,
                            int agent,
                            int insert_time,
                            int count) {
    return insert_actions_for_agent(
        plan,
        num_agents,
        agent,
        insert_time,
        std::vector<Action>(static_cast<std::size_t>(std::max(1, count)), Action::noop())
    );
}

bool states_rejoin_for_agent(const State& start, const State& current, int agent) {
    if (agent < 0 || agent >= start.num_agents() || agent >= current.num_agents()) {
        return false;
    }

    return current.agent_positions[static_cast<std::size_t>(agent)] ==
               start.agent_positions[static_cast<std::size_t>(agent)] &&
           current.box_pos == start.box_pos;
}

bool state_after_prefix(const Level& level,
                        const State& initial_state,
                        const Plan& plan,
                        int prefix_steps,
                        State& out) {
    out = initial_state;

    const int clamped_steps =
        std::max(0, std::min(prefix_steps, static_cast<int>(plan.steps.size())));

    for (int t = 0; t < clamped_steps; ++t) {
        const JointAction& step = plan.steps[static_cast<std::size_t>(t)];

        if (static_cast<int>(step.actions.size()) != out.num_agents()) {
            return false;
        }

        for (int agent = 0; agent < out.num_agents(); ++agent) {
            const Action& action = step.actions[static_cast<std::size_t>(agent)];
            if (!ActionApplicator::is_applicable(level, out, agent, action)) {
                return false;
            }
        }

        for (int agent = 0; agent < out.num_agents(); ++agent) {
            const Action& action = step.actions[static_cast<std::size_t>(agent)];
            out = ActionApplicator::apply(level, out, agent, action);
        }
    }

    return true;
}

bool all_noops(const std::vector<Action>& actions) {
    return std::all_of(actions.begin(), actions.end(), [](const Action& action) {
        return action.type == ActionType::NoOp;
    });
}

void collect_rejoining_detours(const Level& level,
                               const State& start_state,
                               int agent,
                               int max_depth,
                               std::vector<Action>& prefix,
                               State current_state,
                               std::vector<std::vector<Action>>& out,
                               std::size_t max_sequences) {
    if (out.size() >= max_sequences) {
        return;
    }

    if (!prefix.empty() &&
        !all_noops(prefix) &&
        states_rejoin_for_agent(start_state, current_state, agent)) {
        out.push_back(prefix);
        if (out.size() >= max_sequences) {
            return;
        }
    }

    if (static_cast<int>(prefix.size()) >= max_depth) {
        return;
    }

    for (const Action& action : ActionLibrary::ALL_ACTIONS) {
        if (!ActionApplicator::is_applicable(level, current_state, agent, action)) {
            continue;
        }

        State next_state = ActionApplicator::apply(level, current_state, agent, action);
        prefix.push_back(action);
        collect_rejoining_detours(
            level,
            start_state,
            agent,
            max_depth,
            prefix,
            std::move(next_state),
            out,
            max_sequences
        );
        prefix.pop_back();

        if (out.size() >= max_sequences) {
            return;
        }
    }
}

std::vector<std::vector<Action>> rejoining_detours_for_agent(const Level& level,
                                                             const State& initial_state,
                                                             const Plan& plan,
                                                             int agent,
                                                             int insert_time) {
    State start_state;
    if (!state_after_prefix(level, initial_state, plan, insert_time, start_state)) {
        return {};
    }

    constexpr int kMaxDetourDepth = 4;
    constexpr std::size_t kMaxDetourSequences = 64;

    std::vector<std::vector<Action>> detours;
    std::vector<Action> prefix;
    collect_rejoining_detours(
        level,
        start_state,
        agent,
        kMaxDetourDepth,
        prefix,
        start_state,
        detours,
        kMaxDetourSequences
    );

    return detours;
}

bool suffix_is_noop_for_agent(const Plan& plan, int agent, int insert_time) {
    const int clamped_time =
        std::max(0, std::min(insert_time, static_cast<int>(plan.steps.size())));

    for (int t = clamped_time; t < static_cast<int>(plan.steps.size()); ++t) {
        const JointAction& step = plan.steps[static_cast<std::size_t>(t)];
        if (agent >= static_cast<int>(step.actions.size())) {
            continue;
        }
        if (step.actions[static_cast<std::size_t>(agent)].type != ActionType::NoOp) {
            return false;
        }
    }

    return true;
}

std::vector<std::vector<Action>> tail_moves_for_agent(const Level& level,
                                                      const State& initial_state,
                                                      const Plan& plan,
                                                      int agent,
                                                      int insert_time) {
    if (!suffix_is_noop_for_agent(plan, agent, insert_time)) {
        return {};
    }

    State start_state;
    if (!state_after_prefix(level, initial_state, plan, insert_time, start_state)) {
        return {};
    }

    std::vector<std::vector<Action>> moves;
    for (const Action& action : ActionLibrary::ALL_ACTIONS) {
        if (action.type != ActionType::Move) {
            continue;
        }
        if (!ActionApplicator::is_applicable(level, start_state, agent, action)) {
            continue;
        }
        moves.push_back({action});
    }

    return moves;
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
    int inserted_actions{0};
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

        if (a.inserted_actions != b.inserted_actions) {
            return a.inserted_actions > b.inserted_actions;
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
    root.inserted_actions = 0;
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
            result.changed = node.inserted_actions > 0;
            result.plan = node.plan;
            return result;
        }

        const Conflict& conflict = conflicts.front();
        const std::vector<int> agents = affected_agents(conflict, num_agents);

        // If the conflict detector returned a conflict with no responsible agent,
        // this repairer cannot branch meaningfully.
        if (agents.empty()) {
            result.conflict_free = false;
            result.changed = node.inserted_actions > 0;
            result.plan = node.plan;
            result.iterations = expansions;
            return result;
        }

        const std::vector<int> insert_times =
            candidate_insert_times(conflict, node.plan);

        const std::vector<int> wait_counts{1, 2, 4, 8, 16, 32};
        for (int agent : agents) {
            for (int insert_time : insert_times) {
                auto enqueue_candidate = [&](Plan candidate, int inserted_action_count) {
                    if (candidate.steps.size() > max_size) {
                        return;
                    }

                    const std::string candidate_key = plan_key(candidate);
                    if (closed.find(candidate_key) != closed.end()) {
                        return;
                    }

                    // Do not require full executability while conflicts remain; later
                    // conflicting steps may make a best-effort simulation diverge.

                    SearchNode child;
                    child.plan = std::move(candidate);
                    child.inserted_actions = node.inserted_actions + inserted_action_count;
                    child.first_conflict = first_conflict_time(child.plan, initial_state);
                    child.plan_size = static_cast<int>(child.plan.steps.size());
                    child.id = next_id++;

                    open.push(std::move(child));
                };

                for (int wait_count : wait_counts) {
                    enqueue_candidate(
                        insert_noops_for_agent(
                            node.plan,
                            num_agents,
                            agent,
                            insert_time,
                            wait_count
                        ),
                        wait_count
                    );
                }

                auto enqueue_inserted_sequence = [&](const std::vector<Action>& inserted_actions) {
                    enqueue_candidate(
                        insert_actions_for_agent(
                            node.plan,
                            num_agents,
                            agent,
                            insert_time,
                            inserted_actions
                        ),
                        static_cast<int>(inserted_actions.size())
                    );
                };

                for (const std::vector<Action>& detour : rejoining_detours_for_agent(
                         level,
                         initial_state,
                         node.plan,
                         agent,
                         insert_time
                     )) {
                    enqueue_inserted_sequence(detour);
                }

                for (const std::vector<Action>& tail_move : tail_moves_for_agent(
                         level,
                         initial_state,
                         node.plan,
                         agent,
                         insert_time
                     )) {
                    enqueue_inserted_sequence(tail_move);
                }
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

PlanConflictRepairer::Result PlanConflictRepairer::repair(
    const Level& level,
    const State& initial_state,
    const std::vector<AgentPlan>& input,
    int max_iterations,
    int max_extra_steps
) const {
    Result result;
    const int num_agents = initial_state.num_agents();
    std::vector<AgentPlan> plans = input;
    plans.resize(static_cast<std::size_t>(num_agents));
    for (int agent = 0; agent < num_agents; ++agent) {
        AgentPlan& plan = plans[static_cast<std::size_t>(agent)];
        plan.agent = agent;
        if (plan.positions.empty()) {
            plan.positions.push_back(initial_state.agent_positions[static_cast<std::size_t>(agent)]);
        }
    }

    result.agent_plans = plans;
    result.plan = plan_from_agent_plans(plans, num_agents);

    if (num_agents <= 0) {
        result.conflict_free = true;
        return result;
    }

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        result.iterations = iteration;
        const std::vector<Conflict> conflicts = conflicts_for(plans, initial_state);
        if (conflicts.empty()) {
            result.conflict_free = is_executable(level, initial_state, plans);
            result.agent_plans = plans;
            result.plan = plan_from_agent_plans(plans, num_agents);
            return result;
        }

        const Conflict& conflict = conflicts.front();
        const std::vector<int> agents = affected_agents(conflict, num_agents);
        if (agents.empty()) {
            break;
        }

        bool repaired_this_conflict = false;
        for (int agent : agents) {
            std::optional<AgentPlan> replanned = replan_agent_with_constraints(
                level,
                initial_state,
                plans,
                agent,
                conflict,
                max_extra_steps
            );
            if (!replanned.has_value()) {
                continue;
            }

            std::vector<AgentPlan> candidate = plans;
            candidate[static_cast<std::size_t>(agent)] = *replanned;
            const int old_first = first_conflict_time(uncompressed_plan_from_agent_plans(plans, num_agents), initial_state);
            const int new_first = first_conflict_time(uncompressed_plan_from_agent_plans(candidate, num_agents), initial_state);
            if (new_first < old_first && !is_conflict_free(candidate, initial_state)) {
                continue;
            }

            plans = std::move(candidate);
            result.changed = true;
            repaired_this_conflict = true;
            break;
        }

        result.agent_plans = plans;
        result.plan = plan_from_agent_plans(plans, num_agents);
        if (!repaired_this_conflict) {
            break;
        }
    }

    result.conflict_free = is_conflict_free(plans, initial_state) &&
                           is_executable(level, initial_state, plans);
    result.agent_plans = plans;
    result.plan = plan_from_agent_plans(plans, num_agents);
    return result;
}
