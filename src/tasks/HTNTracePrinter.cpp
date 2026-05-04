#include "tasks/HTNTracePrinter.hpp"

#include <ostream>

void HTNTracePrinter::print_task_batch(const std::vector<Task>& tasks, std::ostream& out) {
    out << "[HTN] task_batch size=" << tasks.size() << '\n';

    for (const Task& task : tasks) {
        out << "[HTN] " << task.describe() << '\n';
    }
}
