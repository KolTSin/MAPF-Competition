#pragma once

#include "domain/Level.hpp"
#include "domain/Position.hpp"

#include <unordered_map>
#include <vector>

// Lazily cached BFS distances over the static wall map.
class StaticDistances {
public:
    explicit StaticDistances(const Level& level);
    [[nodiscard]] std::vector<int> bfs_from(Position source) const;
    [[nodiscard]] int distance(Position from, Position to) const;
    void cache_from(Position source);
    [[nodiscard]] bool reachable(Position from, Position to) const;

private:
    const Level& level_;
    mutable std::unordered_map<int, std::vector<int>> cache_;
    [[nodiscard]] int index(Position p) const noexcept;
};
