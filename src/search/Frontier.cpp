#include "search/Frontier.hpp"

#include <stdexcept>

Frontier::Frontier(const std::vector<Node>* nodes)
    : compare_{nodes}, pq_(compare_) {}

bool Frontier::Compare::operator()(int a, int b) const {
    const Node& na = (*nodes)[a];
    const Node& nb = (*nodes)[b];

    // priority_queue puts the "largest" on top, so invert for min-f behavior.
    if (na.f() != nb.f()) {
        return na.f() > nb.f();
    }

    // Tie-break: prefer larger g (deeper node) when f is equal.
    if (na.g != nb.g) {
        return na.g < nb.g;
    }

    return a > b;
}

void Frontier::push(int node_index) {
    pq_.push(node_index);
}

int Frontier::pop() {
    if (pq_.empty()) {
        throw std::runtime_error("Attempted to pop from an empty Frontier");
    }

    const int top = pq_.top();
    pq_.pop();
    return top;
}

bool Frontier::empty() const noexcept {
    return pq_.empty();
}

std::size_t Frontier::size() const noexcept {
    return pq_.size();
}