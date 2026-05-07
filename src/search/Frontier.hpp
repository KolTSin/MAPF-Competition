#pragma once

#include "search/Node.hpp"

#include <queue>
#include <vector>

// Priority queue of indices into a node arena. Storing indices avoids invalidating
// queue entries when the vector grows and keeps copies cheap.
class Frontier {
public:
    explicit Frontier(const std::vector<Node>* nodes);

    void push(int node_index);
    int pop();
    [[nodiscard]] bool empty() const noexcept;
    [[nodiscard]] std::size_t size() const noexcept;

private:
    struct Compare {
        const std::vector<Node>* nodes{nullptr};

        bool operator()(int a, int b) const;
    };

    Compare compare_;
    std::priority_queue<int, std::vector<int>, Compare> pq_;
};
