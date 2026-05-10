#include "tasks/DependencyBuilder.hpp"

#include <algorithm>
#include <unordered_set>

namespace {
bool overlaps(Position a0, Position a1, Position b0, Position b1) {
    return a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1;
}

std::vector<Position> coarse_route(Position from, Position to) {
    std::vector<Position> route;
    Position cur = from;
    route.push_back(cur);
    while (cur.row != to.row) {
        cur.row += (to.row > cur.row) ? 1 : -1;
        route.push_back(cur);
    }
    while (cur.col != to.col) {
        cur.col += (to.col > cur.col) ? 1 : -1;
        route.push_back(cur);
    }
    return route;
}

bool is_agent_only_task(const Task& task) {
    return task.type == TaskType::MoveAgentToGoal || task.type == TaskType::ParkAgentSafely;
}

Position route_end(const Task& task) {
    if (task.type == TaskType::MoveBlockingBoxToParking) return task.parking_pos;
    return task.goal_pos;
}

bool has_route_overlap_risk(const Task& a, const Task& b) {
    const auto ra = coarse_route(a.box_pos, route_end(a));
    const auto rb = coarse_route(b.box_pos, route_end(b));
    for (std::size_t i = 0; i < ra.size(); ++i) {
        for (std::size_t j = 0; j < rb.size(); ++j) {
            if (ra[i] == rb[j]) return true;
            if (i + 1 < ra.size() && j + 1 < rb.size() && ra[i] == rb[j + 1] && ra[i + 1] == rb[j]) return true;
        }
    }
    return false;
}

void add_edge(DependencyGraph& graph, int predecessor, int successor) {
    auto& reverse_preds = graph.predecessors[predecessor];
    if (std::find(reverse_preds.begin(), reverse_preds.end(), successor) != reverse_preds.end()) return;

    auto& preds = graph.predecessors[successor];
    if (std::find(preds.begin(), preds.end(), predecessor) == preds.end()) {
        preds.push_back(predecessor);
        graph.successors[predecessor].push_back(successor);
    }
}
}

std::vector<int> DependencyGraph::ready_tasks(const std::unordered_set<int>& completed) const {
    std::vector<int> ready;
    for (const auto& [task_id, preds] : predecessors) {
        if (completed.count(task_id)) {
            continue;
        }
        bool all_done = true;
        for (int predecessor : preds) {
            if (!completed.count(predecessor)) {
                all_done = false;
                break;
            }
        }
        if (all_done) {
            ready.push_back(task_id);
        }
    }
    return ready;
}

std::vector<int> DependencyGraph::mark_completed_and_get_new_ready(
    int completed_task_id,
    std::unordered_map<int, int>& remaining_predecessors) const {
    std::vector<int> newly_ready;
    auto it = successors.find(completed_task_id);
    if (it == successors.end()) return newly_ready;
    for (int succ : it->second) {
        auto remaining_it = remaining_predecessors.find(succ);
        if (remaining_it == remaining_predecessors.end()) continue;
        remaining_it->second = std::max(0, remaining_it->second - 1);
        if (remaining_it->second == 0) {
            newly_ready.push_back(succ);
        }
    }
    return newly_ready;
}

std::unordered_map<int, std::vector<int>> DependencyBuilder::build(const std::vector<Task>& tasks) const {
    return build_graph(tasks).predecessors;
}

DependencyGraph DependencyBuilder::build_graph(const std::vector<Task>& tasks) const {
    DependencyGraph graph;
    std::unordered_set<int> task_ids;
    for (const Task& task : tasks) {
        graph.predecessors[task.task_id] = {};
        graph.successors[task.task_id] = {};
        task_ids.insert(task.task_id);
    }
    for (const Task& task : tasks) {
        for (int predecessor : task.dependencies) {
            if (task_ids.count(predecessor)) {
                add_edge(graph, predecessor, task.task_id);
            }
        }
    }
    for (std::size_t i = 0; i < tasks.size(); ++i) {
        for (std::size_t j = i + 1; j < tasks.size(); ++j) {
            const Task& a = tasks[i];
            const Task& b = tasks[j];
            const bool a_blocks_same_box = (a.type == TaskType::MoveBlockingBoxToParking && b.box_id == a.box_id);
            const bool b_blocks_same_box = (b.type == TaskType::MoveBlockingBoxToParking && a.box_id == b.box_id);
            if (a_blocks_same_box) {
                add_edge(graph, a.task_id, b.task_id);
            }
            if (b_blocks_same_box) {
                add_edge(graph, b.task_id, a.task_id);
            }
            // When a parking-relocation task and a task for the same box coexist,
            // enforce only relocation -> dependent ordering and skip generic
            // overlap/risk edges that can introduce immediate 2-cycles.
            if (a_blocks_same_box || b_blocks_same_box) {
                continue;
            }
            if (a.type == TaskType::MoveBlockingBoxToParking && b.type == TaskType::DeliverBoxToGoal &&
                a.unblocks_box_id == b.box_id) {
                add_edge(graph, a.task_id, b.task_id);
            }
            if (b.type == TaskType::MoveBlockingBoxToParking && a.type == TaskType::DeliverBoxToGoal &&
                b.unblocks_box_id == a.box_id) {
                add_edge(graph, b.task_id, a.task_id);
            }
            if (a.type == TaskType::DeliverBoxToGoal && b.type == TaskType::MoveAgentToGoal && a.agent_id == b.agent_id) {
                add_edge(graph, a.task_id, b.task_id);
            }
            if (b.type == TaskType::DeliverBoxToGoal && a.type == TaskType::MoveAgentToGoal && a.agent_id == b.agent_id) {
                add_edge(graph, b.task_id, a.task_id);
            }
            // Agent-only cleanup tasks have no box route, so do not feed their
            // placeholder box_pos/parking_pos values into box-route dependency
            // heuristics. Explicit task dependencies above decide when they run.
            if (!is_agent_only_task(a) && !is_agent_only_task(b)) {
                if (overlaps(a.box_pos, a.goal_pos, b.box_pos, b.goal_pos) ||
                    overlaps(a.box_pos, a.parking_pos, b.box_pos, b.parking_pos)) {
                    const int pred = (a.task_id < b.task_id) ? a.task_id : b.task_id;
                    const int succ = (a.task_id < b.task_id) ? b.task_id : a.task_id;
                    add_edge(graph, pred, succ);
                }
                if (has_route_overlap_risk(a, b)) {
                    const int pred = (a.priority > b.priority || (a.priority == b.priority && a.task_id < b.task_id)) ? a.task_id : b.task_id;
                    const int succ = (pred == a.task_id) ? b.task_id : a.task_id;
                    add_edge(graph, pred, succ);
                }
            }
            if (a.type == TaskType::MoveBlockingBoxToParking &&
                b.type == TaskType::DeliverBoxToGoal &&
                overlaps(a.box_pos, a.parking_pos, b.box_pos, b.goal_pos)) {
                add_edge(graph, a.task_id, b.task_id);
            }
        }
    }
    return graph;
}
