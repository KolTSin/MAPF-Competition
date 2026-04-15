#include "search/CBSFrontier.hpp"

#include <stdexcept>

CBSFrontier::CBSFrontier(const std::vector<CBSNode>* nodes)
    : compare_{nodes}, pq_(compare_) {}

bool CBSFrontier::Compare::operator()(int a, int b) const {
    const CBSNode& na = (*nodes)[a];
    const CBSNode& nb = (*nodes)[b];

    // priority_queue puts the "largest" on top, so invert for min-f behavior.
    if (na.sum_of_costs != nb.sum_of_costs) {
        return na.sum_of_costs > nb.sum_of_costs;
    }

    // Tie-break: prefer larger g (deeper node) when f is equal.
    if (na.makespan != nb.makespan) {
        return na.makespan < nb.makespan;
    }

    return a > b;
}

void CBSFrontier::push(int node_index) {
    pq_.push(node_index);
}

int CBSFrontier::pop() {
    if (pq_.empty()) {
        throw std::runtime_error("Attempted to pop from an empty CBSFrontier");
    }

    const int top = pq_.top();
    pq_.pop();
    return top;
}

bool CBSFrontier::empty() const noexcept {
    return pq_.empty();
}

std::size_t CBSFrontier::size() const noexcept {
    return pq_.size();
}