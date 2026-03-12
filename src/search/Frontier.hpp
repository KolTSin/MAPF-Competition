#pragma once

#include "search/Node.hpp"

#include <queue>
#include <vector>

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