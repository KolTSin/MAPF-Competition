#include "search/AStar.hpp"

#include "actions/Action.hpp"
#include "actions/JointAction.hpp"
#include "search/Frontier.hpp"
#include "search/Node.hpp"
#include "search/SuccessorGenerator.hpp"
#include "state/StateEquality.hpp"
#include "state/StateHasher.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <iostream>

namespace {
[[nodiscard]] bool is_goal(const Level& level, const State& state, int agent_id) {
    const Position pos = state.agent_positions[agent_id];
    return level.goal_at(pos.row, pos.col) == static_cast<char>('0' + agent_id);
}

[[nodiscard]] std::vector<Action> reconstruct_plan(const std::vector<Node>& nodes,
                                    int goal_index) {
    std::vector<Action> reversed_steps;

    int current = goal_index;
    while (nodes[current].parent_index != -1) {
        reversed_steps.push_back(nodes[current].action);

        current = nodes[current].parent_index;
    }

    std::reverse(reversed_steps.begin(), reversed_steps.end());

    // Plan plan;
    // plan.steps = std::move(reversed_steps);
    return reversed_steps;
}

} // namespace

std::vector<Action> AStar::search(const Level& level,
                   const State& initial_state,
                   int agent_id) {
    if (agent_id < 0 || agent_id >= initial_state.num_agents()) {
        throw std::runtime_error("Invalid agent_id in AStar::search");
    }
    std::cerr << "Starting search for agent: " << agent_id <<'\n';
    std::vector<Node> nodes;
    nodes.reserve(1024);

    Frontier open(&nodes);

    std::unordered_map<State, int, StateHasher, StateEqual> best_g;
    best_g.reserve(2048);

    Node start;
    start.state = initial_state;
    start.g = 0;
    start.h = heuristic_.evaluate(initial_state);
    start.parent_index = -1;
    start.action = Action::noop();

    nodes.push_back(std::move(start));
    open.push(0);
    best_g[nodes[0].state] = 0;

    std::vector<Successor> successors;
    successors.reserve(29);

    while (!open.empty()) {
        const int current_index = open.pop();
        const Node& current = nodes[current_index];

        auto it_best = best_g.find(current.state);
        if (it_best != best_g.end() && current.g > it_best->second) {
            continue;
        }

        if (is_goal(level, current.state, agent_id)) {
            return reconstruct_plan(nodes,
                                    current_index);
        }

        SuccessorGenerator::expand_agent(level, current.state, agent_id, successors);

        for (const Successor& succ : successors) {
            const int tentative_g = current.g + 1;

            auto it = best_g.find(succ.next_state);
            if (it != best_g.end() && tentative_g >= it->second) {
                continue;
            }

            Node next;
            next.state = succ.next_state;
            next.g = tentative_g;
            next.h = heuristic_.evaluate(next.state);
            next.parent_index = current_index;
            next.action = succ.action;

            nodes.push_back(std::move(next));
            const int next_index = static_cast<int>(nodes.size()) - 1;

            best_g[nodes[next_index].state] = tentative_g;
            open.push(next_index);
        }
    }

    return std::vector<Action>{};
}