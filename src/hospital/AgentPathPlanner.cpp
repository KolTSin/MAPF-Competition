#include "hospital/AgentPathPlanner.hpp"

#include "search/SuccessorGenerator.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

struct AgentKey {
    Position pos;
    int time;

    bool operator==(const AgentKey& other) const noexcept {
        return pos == other.pos && time == other.time;
    }
};

struct AgentKeyHash {
    std::size_t operator()(const AgentKey& key) const noexcept {
        return (static_cast<std::size_t>(key.pos.row) * 73856093u) ^
               (static_cast<std::size_t>(key.pos.col) * 19349663u) ^
               (static_cast<std::size_t>(key.time) * 83492791u);
    }
};

struct AgentNode {
    Position pos;
    int g{0};
    int h{0};
    int time{0};
    int parent{-1};
    Action action{Action::noop()};

    [[nodiscard]] int f() const noexcept { return g + h; }
};

int manhattan(Position a, Position b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

TaskPlan reconstruct_task_plan(const Task& task, const std::vector<AgentNode>& nodes, int goal_index) {
    TaskPlan out;
    out.task_id = task.task_id;
    out.task_type = task.type;
    out.agent_id = task.agent_id;
    out.agent_plan.agent = task.agent_id;

    std::vector<Action> reversed_actions;
    std::vector<Position> reversed_positions;

    int cur = goal_index;
    reversed_positions.push_back(nodes[cur].pos);
    while (nodes[cur].parent != -1) {
        reversed_actions.push_back(nodes[cur].action);
        cur = nodes[cur].parent;
        reversed_positions.push_back(nodes[cur].pos);
    }

    std::reverse(reversed_actions.begin(), reversed_actions.end());
    std::reverse(reversed_positions.begin(), reversed_positions.end());

    out.agent_plan.actions = std::move(reversed_actions);
    out.agent_plan.positions = std::move(reversed_positions);
    out.box_trajectory.assign(out.agent_plan.positions.size(), task.box_pos);
    out.success = true;
    return out;
}

} // namespace

TaskPlan AgentPathPlanner::plan(const Level& level, const State& state, const Task& task, const ReservationTable& reservations) const {
    return plan(level, state, task, reservations, 0);
}

TaskPlan AgentPathPlanner::plan(const Level& level, const State& state, const Task& task, const ReservationTable& reservations, int start_time) const {
    TaskPlan out;
    out.task_id = task.task_id;
    out.task_type = task.type;
    out.agent_id = task.agent_id;
    out.agent_plan.agent = task.agent_id;

    if (task.agent_id < 0 || task.agent_id >= state.num_agents()) {
        out.failure_reason = "invalid_agent";
        return out;
    }
    if (!level.in_bounds(task.goal_pos.row, task.goal_pos.col) || level.is_wall(task.goal_pos.row, task.goal_pos.col)) {
        out.failure_reason = "invalid_agent_target";
        return out;
    }
    const Position start = state.agent_positions[task.agent_id];
    if (!reservations.can_occupy_agent(task.agent_id, start, start_time)) {
        out.failure_reason = "reserved_start_cell";
        return out;
    }

    std::vector<AgentNode> nodes;
    nodes.reserve(1024);

    auto cmp = [&nodes](int a, int b) {
        const AgentNode& lhs = nodes[a];
        const AgentNode& rhs = nodes[b];
        if (lhs.f() != rhs.f()) return lhs.f() > rhs.f();
        if (lhs.h != rhs.h) return lhs.h > rhs.h;
        return lhs.g < rhs.g;
    };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);
    std::unordered_map<AgentKey, int, AgentKeyHash> best_g;
    best_g.reserve(1024);

    nodes.push_back(AgentNode{start, 0, manhattan(start, task.goal_pos), 0, -1, Action::noop()});
    open.push(0);
    best_g[AgentKey{start, 0}] = 0;

    const int max_time = std::max(64, level.rows * level.cols * 4);
    std::vector<AgentTaskSuccessor> successors;
    successors.reserve(5);

    while (!open.empty()) {
        const int current_index = open.top();
        open.pop();
        const AgentNode current = nodes[current_index];

        const auto it_best = best_g.find(AgentKey{current.pos, current.time});
        if (it_best != best_g.end() && current.g > it_best->second) continue;

        if (current.pos == task.goal_pos) {
            return reconstruct_task_plan(task, nodes, current_index);
        }
        if (current.time >= max_time) continue;

        SuccessorGenerator::expand_agent_task(level,
                                              state,
                                              task.agent_id,
                                              current.pos,
                                              current.time,
                                              start_time,
                                              reservations,
                                              successors);
        for (const AgentTaskSuccessor& succ : successors) {
            AgentNode next;
            next.pos = succ.next_agent;
            next.g = current.g + 1;
            next.h = manhattan(next.pos, task.goal_pos);
            next.time = succ.next_time;
            next.parent = current_index;
            next.action = succ.action;

            AgentKey key{next.pos, next.time};
            auto it = best_g.find(key);
            if (it != best_g.end() && next.g >= it->second) continue;

            nodes.push_back(next);
            const int next_index = static_cast<int>(nodes.size()) - 1;
            best_g[key] = next.g;
            open.push(next_index);
        }
    }

    out.failure_reason = "no_path_for_agent_reposition";
    return out;
}
