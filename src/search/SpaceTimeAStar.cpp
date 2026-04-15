#include "search/SpaceTimeAStar.hpp"
#include "search/Node.hpp"
#include "search/SuccessorGenerator.hpp"
#include "state/StateEquality.hpp"
#include "state/StateHasher.hpp"

#include <algorithm>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace {

struct SpaceTimeKey {
    State state;
    int time;

    bool operator==(const SpaceTimeKey& other) const noexcept {
        return time == other.time &&
               StateEqual{}(state, other.state);
    }
};

struct SpaceTimeKeyHasher {
    std::size_t operator()(const SpaceTimeKey& k) const noexcept {
        std::size_t seed = StateHasher{}(k.state);

        auto hash_combine = [](std::size_t& s, std::size_t value) {
            s ^= value + 0x9e3779b9 + (s << 6) + (s >> 2);
        };

        hash_combine(seed, std::hash<int>{}(k.time));
        return seed;
    }
};

bool has_agent_goal(const Level& level, int agent_id) {
    const char goal_char = static_cast<char>('0' + agent_id);
    for (char goal : level.goals){
        if (goal == goal_char){
            return true;
        }
    }
    return false;
}

[[nodiscard]] bool is_goal(const Level& level, const State& state, int agent_id) {
    const Position pos = state.agent_positions[agent_id];
    return level.goal_at(pos.row, pos.col) == static_cast<char>('0' + agent_id);
}

AgentPlan reconstruct_plan(const std::vector<Node>& nodes, int goal_index, int agent) {
    AgentPlan reversed_steps;

    int current = goal_index;
    while (nodes[current].parent_index != -1) {
        reversed_steps.add(nodes[current].action,nodes[current].state.agent_positions[agent]);

        current = nodes[current].parent_index;
    }
    reversed_steps.positions.push_back(nodes[current].state.agent_positions[agent]);
    reversed_steps.agent = agent;

    std::reverse(reversed_steps.actions.begin(), reversed_steps.actions.end());
    std::reverse(reversed_steps.positions.begin(), reversed_steps.positions.end());

    // Plan plan;
    // plan.steps = std::move(reversed_steps);
    return reversed_steps;
}

class OpenList {
public:
    explicit OpenList(const std::vector<Node>* nodes)
        : nodes_(nodes) {}

    void push(int index) {
        pq_.push(index);
    }

    int pop() {
        int top = pq_.top();
        pq_.pop();
        return top;
    }

    bool empty() const noexcept {
        return pq_.empty();
    }

private:
    struct Compare {
        const std::vector<Node>* nodes{nullptr};

        bool operator()(int a, int b) const {
            const auto& na = (*nodes)[a];
            const auto& nb = (*nodes)[b];

            if (na.f() != nb.f()) {
                return na.f() > nb.f();
            }
            if (na.g != nb.g) {
                return na.g < nb.g;
            }
            return a > b;
        }
    };

    const std::vector<Node>* nodes_;
    std::priority_queue<int, std::vector<int>, Compare> pq_{Compare{nodes_}};
};

bool is_move_valid(const Level& level,
                   const ReservationTable& reservations,
                   const Position& current,
                   const Position& next,
                   int current_time,
                   int agent) {
    if (!level.in_bounds(next.row, next.col)) {
        return false;
    }

    if (level.is_wall(next.row, next.col)) {
        return false;
    }

    const int next_time = current_time + 1;

    // Follow-conflict prevention:
    // destination cannot be occupied at the start of the timestep
    if (reservations.is_cell_reserved(next.row, next.col, current_time, agent) || 
        reservations.is_cell_reserved(next.row, next.col, next_time, agent) || 
        reservations.is_cell_reserved(next.row, next.col, next_time + 1, agent)) {
        // std::cerr << "follow conflict!" << '\n';
        return false;
    }

    // Vertex reservation
    if (reservations.is_cell_reserved(next.row, next.col, next_time, agent)) {
        // std::cerr << "vertex conflict!" << '\n';
        return false;
    }

    // Edge swap prevention:
    // If someone else already reserved next -> current at current_time,
    // then current -> next is forbidden.
    if (reservations.is_edge_reserved(next, current, current_time, agent)) {
        // std::cerr << "edge conflict!" << '\n';
        return false;
    }
    // if (agent == 2){
    //     std::cerr << "No conflict detected! " 
    //     << current_time << " : " << reservations.is_cell_reserved(next.row, next.col, current_time, agent) 
    //     << next_time << " : " << reservations.is_cell_reserved(next.row, next.col, next_time, agent) << '\n';
    // }
    return true;
}

} // namespace

AgentPlan SpaceTimeAStar::search(
    const Level& level,
    const State& initial_state,
    const int agent,
    const int max_time,
    const ReservationTable& reservations
) {
    if (!has_agent_goal(level, agent)) {
        return {};
    }
    std::vector<Node> nodes;
    nodes.reserve(2048);

    OpenList open(&nodes);

    std::unordered_map<SpaceTimeKey, int, SpaceTimeKeyHasher> best_g;
    best_g.reserve(2048);

    Node start_node;
    start_node.state = initial_state;
    start_node.time = 0;
    start_node.g = 0;
    start_node.h = heuristic_.evaluate(initial_state);
    start_node.parent_index = -1;
    start_node.action = Action::noop();

    nodes.push_back(start_node);
    open.push(0);
    best_g[{start_node.state, 0}] = 0;

    std::vector<Successor> successors;
    successors.reserve(29);

    while (!open.empty()) {
        int current_index = open.pop();
        Node current = nodes[current_index];

        SpaceTimeKey current_key{current.state, current.time};
        auto it_best = best_g.find(current_key);
        if (it_best != best_g.end() && current.g > it_best->second) {
            continue;
        }

        if (is_goal(level, current.state, agent)) {
            return reconstruct_plan(nodes, current_index, agent);
        }

        SuccessorGenerator::expand_agent(level, current.state, agent, current.time, successors);
        try {
            for (const Successor& succ : successors) {
                // std::cerr << "trying move: " << succ.action.to_string() << '\n';

                Position next_pos = succ.next_state.agent_positions[agent];

                if (!is_move_valid(level, reservations, current.state.agent_positions[agent], next_pos, current.time, agent)) {
                    continue;
                }

                
                Node next;
                next.state = succ.next_state;
                next.time = current.time + 1;
                next.g = current.g + 1;
                next.h = heuristic_.evaluate(next.state);
                next.parent_index = current_index;
                next.action = succ.action;

                SpaceTimeKey key{next.state, next.time};

                auto it = best_g.find(key);
                if (next.time == max_time){
                    continue;
                }
                if (it != best_g.end() && next.g >= it->second) {
                    continue;
                }
                // if (agent == 2){
                //     std::cerr << "found next move: " << succ.action.to_string() << '\n';
                // }
                nodes.push_back(next);
                int next_index = static_cast<int>(nodes.size()) - 1;
                best_g[key] = next.g;
                open.push(next_index);
                
            }
        } catch (const std::exception& e) {
            throw;
        }
        // std::cerr << "finished searching for agent " << agent << '\n';
    }
    return AgentPlan{};
}
