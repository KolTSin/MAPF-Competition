#pragma once

#include "tasks/Task.hpp"

#include <unordered_map>
#include <vector>

class DependencyBuilder {
public:
    [[nodiscard]] std::unordered_map<int, std::vector<int>> build(const std::vector<Task>& tasks) const;
};
