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
#include <iostream>

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

[[nodiscard]] bool is_goal(const Level& level, const State& state, int agent_id) {
    const Position pos = state.agent_positions[agent_id];
    return level.goal_at(pos.row, pos.col) == static_cast<char>('0' + agent_id);
}

[[nodiscard]] bool level_has_agent_goal(const Level& level, int agent_id) {
    const char goal_symbol = static_cast<char>('0' + agent_id);
    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            if (level.goal_at(r, c) == goal_symbol) return true;
        }
    }
    return false;
}

std::vector<Action> reconstruct_plan(const std::vector<Node>& nodes, int goal_index) {
    std::vector<Action> reversed;

    int current = goal_index;
    while (nodes[current].parent_index != -1) {
        reversed.push_back(nodes[current].action);
        current = nodes[current].parent_index;
    }

    std::reverse(reversed.begin(), reversed.end());
    return reversed;
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

    // Space-time planning treats the same board position at different timesteps
    // as different nodes, then filters each transition through reservations.

    // Follow-conflict prevention:
    // destination cannot be occupied at the start of the timestep
    if (reservations.is_cell_reserved(next.row, next.col, current_time, agent) || 
        reservations.is_cell_reserved(next.row, next.col, next_time, agent) || 
        reservations.is_cell_reserved(next.row, next.col, next_time + 1, agent)) {
        std::cerr << "follow conflict!" << '\n';
        return false;
    }

    // Vertex reservation
    if (reservations.is_cell_reserved(next.row, next.col, next_time, agent)) {
        std::cerr << "vertex conflict!" << '\n';
        return false;
    }

    // Edge swap prevention:
    // If someone else already reserved next -> current at current_time,
    // then current -> next is forbidden.
    if (reservations.is_edge_reserved(next, current, current_time, agent)) {
        std::cerr << "edge conflict!" << '\n';
        return false;
    }
    if (agent == 2){
        std::cerr << "No conflict detected! " 
        << current_time << " : " << reservations.is_cell_reserved(next.row, next.col, current_time, agent) 
        << next_time << " : " << reservations.is_cell_reserved(next.row, next.col, next_time, agent) << '\n';
    }
    return true;
}

} // namespace

std::vector<Action> SpaceTimeAStar::search(
    const Level& level,
    const State& initial_state,
    const int agent,
    const ReservationTable& reservations
) {
    if (!level_has_agent_goal(level, agent)) {
        // Some MAPF levels only contain box goals. In that case this single-agent
        // goal search has nothing useful to do.
        return {};
    }

    std::vector<Node> nodes;
    nodes.reserve(1024);

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

    std::size_t expansions = 0;
    std::size_t generated = 0;

    while (!open.empty()) {
        int current_index = open.pop();
        const Node& current = nodes[current_index];

        SpaceTimeKey current_key{current.state, current.time};
        auto it_best = best_g.find(current_key);
        if (it_best != best_g.end() && current.g > it_best->second) {
            // This queue entry was made obsolete by a cheaper path to the same
            // state at the same time.
            continue;
        }

        if (is_goal(level, current.state, agent)) {
            std::cerr << "agent " << agent
                    << " reached goal after expansions=" << expansions
                    << ", nodes=" << nodes.size()
                    << ", best_g=" << best_g.size()
                    << '\n';
            return reconstruct_plan(nodes, current_index);
        }

        SuccessorGenerator::expand_agent(level, current.state, agent, current.time, successors);
        generated += successors.size();
        try {
            for (const Successor& succ : successors) {
                // std::cerr << "trying move: " << succ.action.to_string() << '\n';

                ++expansions;
                if (expansions % 100 == 0) {
                    std::cerr << "agent=" << agent
                            << " expansions=" << expansions
                            << " nodes=" << nodes.size()
                            << " best_g=" << best_g.size()
                            << " current_time=" << current.time
                            << '\n';
                }

                Position next_pos = succ.next_state.agent_positions[agent];

                // Reservation checks reject collisions with already committed
                // plans before the successor enters the open list.
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
                if (it != best_g.end() && next.g >= it->second) {
                    continue;
                }
                if (agent == 2){
                    std::cerr << "found next move: " << succ.action.to_string() << '\n';
                }
                nodes.push_back(next);
                int next_index = static_cast<int>(nodes.size()) - 1;
                best_g[key] = next.g;
                open.push(next_index);
                
            }
        } catch (const std::exception& e) {
            std::cerr << "AStar failed for agent " << agent
                    << " with exception: " << e.what() << '\n';
            throw;
        }
        // std::cerr << "finished searching for agent " << agent << '\n';
    }
    std::cerr << "agent " << agent
          << " search failed: open list exhausted"
          << ", expansions=" << expansions
          << ", generated=" << generated
          << ", nodes=" << nodes.size()
          << ", best_g=" << best_g.size()
          << '\n';
    return std::vector<Action>{};
}
