#pragma once

#include "tasks/Task.hpp"

#include <unordered_set>
#include <unordered_map>
#include <vector>

struct DependencyGraph {
    std::unordered_map<int, std::vector<int>> predecessors;
    std::unordered_map<int, std::vector<int>> successors;

    [[nodiscard]] std::vector<int> ready_tasks(const std::unordered_set<int>& completed) const;
    [[nodiscard]] std::vector<int> mark_completed_and_get_new_ready(
        int completed_task_id,
        std::unordered_map<int, int>& remaining_predecessors) const;
};

class DependencyBuilder {
public:
    [[nodiscard]] std::unordered_map<int, std::vector<int>> build(const std::vector<Task>& tasks) const;
    [[nodiscard]] DependencyGraph build_graph(const std::vector<Task>& tasks) const;
};
