#pragma once

#include "search/CBSNode.hpp"

#include <queue>
#include <vector>

class CBSFrontier {
public:
    explicit CBSFrontier(const std::vector<CBSNode>* nodes);

    void push(int node_index);
    int pop();
    [[nodiscard]] bool empty() const noexcept;
    [[nodiscard]] std::size_t size() const noexcept;

private:
    struct Compare {
        const std::vector<CBSNode>* nodes{nullptr};

        bool operator()(int a, int b) const;
    };

    Compare compare_;
    std::priority_queue<int, std::vector<int>, Compare> pq_;
};