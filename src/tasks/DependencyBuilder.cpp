#include "tasks/DependencyBuilder.hpp"

#include <algorithm>
#include <unordered_set>

namespace {
void add_edge(DependencyGraph& graph, int predecessor, int successor) {
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
    for (const Task& task : tasks) {
        graph.predecessors[task.task_id] = {};
        graph.successors[task.task_id] = {};
    }
    for (const Task& a : tasks) {
        for (const Task& b : tasks) {
            if (a.task_id == b.task_id) continue;
            if (a.type == TaskType::MoveBlockingBoxToParking && b.box_id == a.box_id) {
                add_edge(graph, a.task_id, b.task_id);
            }
            if (a.type == TaskType::DeliverBoxToGoal && b.type == TaskType::MoveAgentToGoal && a.agent_id == b.agent_id) {
                add_edge(graph, a.task_id, b.task_id);
            }
        }
    }
    return graph;
}
