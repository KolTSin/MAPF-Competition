#pragma once

#include "tasks/Task.hpp"

#include <iosfwd>
#include <vector>

class HTNTracePrinter {
public:
    static void print_task_batch(const std::vector<Task>& tasks, std::ostream& out);
};
