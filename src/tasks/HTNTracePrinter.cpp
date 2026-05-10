#include "tasks/HTNTracePrinter.hpp"

#include <algorithm>
#include <ostream>

void HTNTracePrinter::print_task_batch(const std::vector<Task>& tasks, std::ostream& out) {
    out << "[HTN] task_batch size=" << tasks.size() << '\n';

    std::vector<Task> ordered = tasks;
    std::sort(ordered.begin(), ordered.end(), [](const Task& a, const Task& b) {
        if (a.priority != b.priority) return a.priority > b.priority;
        return a.task_id < b.task_id;
    });

    for (const Task& task : ordered) {
        out << "[HTN] " << task.describe() << '\n';
    }
}
