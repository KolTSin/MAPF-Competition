#include "tasks/DependencyBuilder.hpp"

std::unordered_map<int, std::vector<int>> DependencyBuilder::build(const std::vector<Task>& tasks) const {
    std::unordered_map<int, std::vector<int>> deps;
    for (const Task& task : tasks) deps[task.task_id] = {};

    for (const Task& a : tasks) {
        for (const Task& b : tasks) {
            if (a.task_id == b.task_id) continue;
            if (a.type == TaskType::MoveBlockingBoxToParking && b.box_id == a.box_id) {
                deps[b.task_id].push_back(a.task_id);
            }
            if (a.type == TaskType::DeliverBoxToGoal && b.type == TaskType::MoveAgentToGoal && a.agent_id == b.agent_id) {
                deps[b.task_id].push_back(a.task_id);
            }
        }
    }
    return deps;
}
