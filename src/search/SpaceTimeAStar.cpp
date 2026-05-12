#include "search/SpaceTimeAStar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>

namespace {

struct PositionTimeKey {
    Position pos;
    int time{0};

    bool operator==(const PositionTimeKey& other) const noexcept {
        return pos == other.pos && time == other.time;
    }
};

struct PositionTimeKeyHasher {
    std::size_t operator()(const PositionTimeKey& k) const noexcept {
        std::size_t seed = PositionHash{}(k.pos);
        seed ^= std::hash<int>{}(k.time) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

bool has_agent_goal(const Level& level, int agent_id) {
    const char goal_char = static_cast<char>('0' + agent_id);
    for (char goal : level.goals) {
        if (goal == goal_char) return true;
    }
    return false;
}

Position find_agent_goal(const Level& level, int agent_id) {
    const char goal_char = static_cast<char>('0' + agent_id);
    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            if (level.goal_at(r, c) == goal_char) {
                return Position{r, c};
            }
        }
    }
    return Position{-1, -1};
}

int manhattan(Position a, Position b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

Action move_action(Position from, Position to) {
    const int dr = to.row - from.row;
    const int dc = to.col - from.col;
    if (dr == 0 && dc == 0) return Action::noop();
    if (dr == -1 && dc == 0) return Action::move(Direction::North);
    if (dr == 1 && dc == 0) return Action::move(Direction::South);
    if (dr == 0 && dc == 1) return Action::move(Direction::East);
    if (dr == 0 && dc == -1) return Action::move(Direction::West);
    return Action::noop();
}

bool is_static_free(const Level& level, const State& initial_state, Position pos) {
    if (!level.in_bounds(pos.row, pos.col)) return false;
    if (level.is_wall(pos.row, pos.col)) return false;
    return !initial_state.has_box(pos.row, pos.col);
}

bool is_move_valid(const ReservationTable& reservations,
                   const Position& current,
                   const Position& next,
                   int current_time,
                   int agent) {
    const int next_time = current_time + 1;

    // CBS constraints are agent-specific prohibitions. A vertex constraint bans
    // being at the destination at the constrained timestep; edge constraints ban
    // exactly the selected transition at its departure timestep. Other agents'
    // initial positions must not be treated as permanent obstacles here: CBS is
    // responsible for resolving their time-dependent interactions.
    if (reservations.is_cell_reserved(next.row, next.col, next_time, agent)) {
        return false;
    }
    if (reservations.is_edge_reserved(next, current, current_time, agent)) {
        return false;
    }
    return true;
}

AgentPlan reconstruct_plan(const std::vector<SpaceTimeNode>& nodes, int goal_index, int agent) {
    AgentPlan reversed_steps;
    reversed_steps.agent = agent;

    int current = goal_index;
    while (nodes[current].parent_index != -1) {
        reversed_steps.add(nodes[current].action, nodes[current].pos);
        current = nodes[current].parent_index;
    }
    reversed_steps.positions.push_back(nodes[current].pos);

    std::reverse(reversed_steps.actions.begin(), reversed_steps.actions.end());
    std::reverse(reversed_steps.positions.begin(), reversed_steps.positions.end());
    return reversed_steps;
}

class OpenList {
public:
    explicit OpenList(const std::vector<SpaceTimeNode>* nodes)
        : nodes_(nodes) {}

    void push(int index) { pq_.push(index); }

    int pop() {
        const int top = pq_.top();
        pq_.pop();
        return top;
    }

    bool empty() const noexcept { return pq_.empty(); }

private:
    struct Compare {
        const std::vector<SpaceTimeNode>* nodes{nullptr};

        bool operator()(int a, int b) const {
            const SpaceTimeNode& na = (*nodes)[a];
            const SpaceTimeNode& nb = (*nodes)[b];
            if (na.f() != nb.f()) return na.f() > nb.f();
            if (na.h != nb.h) return na.h > nb.h;
            if (na.g != nb.g) return na.g < nb.g;
            return a > b;
        }
    };

    const std::vector<SpaceTimeNode>* nodes_;
    std::priority_queue<int, std::vector<int>, Compare> pq_{Compare{nodes_}};
};

} // namespace

AgentPlan SpaceTimeAStar::search(
    const Level& level,
    const State& initial_state,
    const int agent,
    const int max_time,
    const ReservationTable& reservations
) {
    if (agent < 0 || agent >= initial_state.num_agents()) {
        return AgentPlan{};
    }

    if (!has_agent_goal(level, agent)) {
        AgentPlan stationary_plan;
        stationary_plan.agent = agent;
        stationary_plan.positions.push_back(initial_state.agent_positions[agent]);
        return stationary_plan;
    }

    const Position goal = find_agent_goal(level, agent);
    if (goal.row < 0 || goal.col < 0) {
        return AgentPlan{};
    }

    std::vector<SpaceTimeNode> nodes;
    nodes.reserve(2048);

    OpenList open(&nodes);
    std::unordered_map<PositionTimeKey, int, PositionTimeKeyHasher> best_g;
    best_g.reserve(2048);

    SpaceTimeNode start_node;
    start_node.pos = initial_state.agent_positions[agent];
    start_node.time = 0;
    start_node.g = 0;
    start_node.h = manhattan(start_node.pos, goal);
    start_node.parent_index = -1;
    start_node.action = Action::noop();

    nodes.push_back(start_node);
    open.push(0);
    best_g[{start_node.pos, 0}] = 0;

    constexpr std::array<Position, 5> kDeltas{{
        Position{0, 0},
        Position{-1, 0},
        Position{1, 0},
        Position{0, 1},
        Position{0, -1},
    }};

    while (!open.empty()) {
        const int current_index = open.pop();
        const SpaceTimeNode current = nodes[current_index];

        const PositionTimeKey current_key{current.pos, current.time};
        const auto it_best = best_g.find(current_key);
        if (it_best != best_g.end() && current.g > it_best->second) {
            continue;
        }

        if (current.pos == goal) {
            return reconstruct_plan(nodes, current_index, agent);
        }

        if (current.time >= max_time) {
            continue;
        }

        for (const Position& delta : kDeltas) {
            const Position next_pos{current.pos.row + delta.row, current.pos.col + delta.col};
            if (!is_static_free(level, initial_state, next_pos)) {
                continue;
            }
            if (!is_move_valid(reservations, current.pos, next_pos, current.time, agent)) {
                continue;
            }

            SpaceTimeNode next;
            next.pos = next_pos;
            next.time = current.time + 1;
            next.g = current.g + 1;
            next.h = manhattan(next_pos, goal);
            next.parent_index = current_index;
            next.action = move_action(current.pos, next_pos);

            const PositionTimeKey key{next.pos, next.time};
            const auto it = best_g.find(key);
            if (it != best_g.end() && next.g >= it->second) {
                continue;
            }

            nodes.push_back(next);
            const int next_index = static_cast<int>(nodes.size()) - 1;
            best_g[key] = next.g;
            open.push(next_index);
        }
    }

    return AgentPlan{};
}
