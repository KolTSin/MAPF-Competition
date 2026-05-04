#include "analysis/StaticDistances.hpp"
#include <queue>

StaticDistances::StaticDistances(const Level& level) : level_(level) {}
int StaticDistances::index(Position p) const noexcept { return level_.index(p.row, p.col); }
std::vector<int> StaticDistances::bfs_from(Position source) const {
    std::vector<int> dist(static_cast<std::size_t>(level_.rows * level_.cols), -1);
    if (level_.is_wall(source.row, source.col)) return dist;
    std::queue<Position> q; q.push(source); dist[static_cast<std::size_t>(index(source))] = 0;
    constexpr int DR[4] = {-1, 1, 0, 0}; constexpr int DC[4] = {0, 0, -1, 1};
    while (!q.empty()) {
        const Position cur = q.front(); q.pop();
        for (int i = 0; i < 4; ++i) {
            Position nxt{cur.row + DR[i], cur.col + DC[i]};
            if (!level_.in_bounds(nxt.row, nxt.col) || level_.is_wall(nxt.row, nxt.col)) continue;
            const int ni = index(nxt); const int ci = index(cur);
            if (dist[static_cast<std::size_t>(ni)] != -1) continue;
            dist[static_cast<std::size_t>(ni)] = dist[static_cast<std::size_t>(ci)] + 1;
            q.push(nxt);
        }
    }
    return dist;
}
void StaticDistances::cache_from(Position source) { const int key = index(source); if (!cache_.contains(key)) cache_[key] = bfs_from(source); }
int StaticDistances::distance(Position from, Position to) const { const int key = index(from); if (!cache_.contains(key)) cache_[key] = bfs_from(from); return cache_.at(key)[static_cast<std::size_t>(index(to))]; }
bool StaticDistances::reachable(Position from, Position to) const { return distance(from, to) >= 0; }
