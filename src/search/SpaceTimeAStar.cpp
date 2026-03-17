#include "search/SpaceTimeAStar.hpp"

#include <algorithm>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace {

struct SpaceTimeKey {
    int row;
    int col;
    int time;

    bool operator==(const SpaceTimeKey& other) const noexcept {
        return row == other.row &&
               col == other.col &&
               time == other.time;
    }
};

struct SpaceTimeKeyHasher {
    std::size_t operator()(const SpaceTimeKey& k) const noexcept {
        std::size_t seed = 0;

        auto hash_combine = [](std::size_t& s, std::size_t value) {
            s ^= value + 0x9e3779b9 + (s << 6) + (s >> 2);
        };

        hash_combine(seed, std::hash<int>{}(k.row));
        hash_combine(seed, std::hash<int>{}(k.col));
        hash_combine(seed, std::hash<int>{}(k.time));
        return seed;
    }
};

bool is_goal(const SpaceTimeNode& node, const Position& goal) {
    return node.pos == goal;
}

std::vector<Action> reconstruct_plan(const std::vector<SpaceTimeNode>& nodes, int goal_index) {
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
    explicit OpenList(const std::vector<SpaceTimeNode>* nodes)
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
        const std::vector<SpaceTimeNode>* nodes{nullptr};

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

    const std::vector<SpaceTimeNode>* nodes_;
    std::priority_queue<int, std::vector<int>, Compare> pq_{Compare{nodes_}};
};

bool is_move_valid(const Level& level,
                   const ReservationTable& reservations,
                   const Position& current,
                   const Position& next,
                   int current_time) {
    if (!level.in_bounds(next.row, next.col)) {
        return false;
    }

    if (level.is_wall(next.row, next.col)) {
        return false;
    }

    const int next_time = current_time + 1;

    // Vertex reservation
    if (reservations.is_cell_reserved(next.row, next.col, next_time)) {
        return false;
    }

    // Edge swap prevention:
    // If someone else already reserved next -> current at current_time,
    // then current -> next is forbidden.
    if (reservations.is_edge_reserved(next, current, current_time)) {
        return false;
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
    std::vector<SpaceTimeNode> nodes;
    nodes.reserve(1024);

    OpenList open(&nodes);

    std::unordered_map<SpaceTimeKey, int, SpaceTimeKeyHasher> best_g;
    best_g.reserve(2048);

    Position start = initial_state.agent_positions[agent];

    SpaceTimeNode start_node;
    start_node.pos = start;
    start_node.time = 0;
    start_node.g = 0;
    start_node.h = heuristic_.evaluate(initial_state);
    start_node.parent_index = -1;
    start_node.action = Action::noop();

    nodes.push_back(start_node);
    open.push(0);
    best_g[{start.row, start.col, 0}] = 0;

    const Action moves[5] = {
        Action::noop(),
        Action::move(Direction::North),
        Action::move(Direction::South),
        Action::move(Direction::East),
        Action::move(Direction::West)
    };

    while (!open.empty()) {
        int current_index = open.pop();
        const SpaceTimeNode& current = nodes[current_index];

        SpaceTimeKey current_key{current.pos.row, current.pos.col, current.time};
        auto it_best = best_g.find(current_key);
        if (it_best != best_g.end() && current.g > it_best->second) {
            continue;
        }

        if (is_goal(current, goal)) {
            return reconstruct_plan(nodes, current_index);
        }

        for (const Action& action : moves) {
            Position next_pos = current.pos;

            if (action.type == ActionType::Move) {
                next_pos.row += drow(action.move_dir);
                next_pos.col += dcol(action.move_dir);
            }

            if (action.type == ActionType::NoOp) {
                // still must obey reservations for staying in place
                if (reservations.is_cell_reserved(next_pos.row, next_pos.col, current.time + 1)) {
                    continue;
                }
            } else {
                if (!is_move_valid(level, reservations, current.pos, next_pos, current.time)) {
                    continue;
                }
            }

            SpaceTimeNode next;
            next.pos = next_pos;
            next.time = current.time + 1;
            next.g = current.g + 1;
            next.h = heuristic(next_pos, goal);
            next.parent_index = current_index;
            next.action = action;

            SpaceTimeKey key{next.pos.row, next.pos.col, next.time};

            auto it = best_g.find(key);
            if (it != best_g.end() && next.g >= it->second) {
                continue;
            }

            nodes.push_back(next);
            int next_index = static_cast<int>(nodes.size()) - 1;
            best_g[key] = next.g;
            open.push(next_index);
        }
    }

    return {};
}