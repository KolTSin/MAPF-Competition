#include "hospital/MapAnalysis.hpp"

#include <algorithm>
#include <queue>

namespace {
constexpr int DR[4] = {-1, 1, 0, 0};
constexpr int DC[4] = {0, 0, -1, 1};
}

MapAnalysis::MapAnalysis(const Level& level)
    : level_(level),
      degree_map_(static_cast<std::size_t>(level.rows * level.cols), 0) {
    for (int row = 0; row < level.rows; ++row) {
        for (int col = 0; col < level.cols; ++col) {
            if (!is_walkable(row, col)) {
                continue;
            }

            int degree = 0;
            for (int k = 0; k < 4; ++k) {
                const int nr = row + DR[k];
                const int nc = col + DC[k];
                if (is_walkable(nr, nc)) {
                    ++degree;
                }
            }

            degree_map_[index(row, col)] = degree;
        }
    }
}

bool MapAnalysis::is_walkable(const int row, const int col) const noexcept {
    return level_.in_bounds(row, col) && !level_.is_wall(row, col);
}

int MapAnalysis::degree(const int row, const int col) const noexcept {
    if (!is_walkable(row, col)) {
        return 0;
    }
    return degree_map_[index(row, col)];
}

bool MapAnalysis::is_chokepoint(const int row, const int col) const noexcept {
    if (!is_walkable(row, col)) {
        return false;
    }

    const int deg = degree(row, col);
    return deg <= 2;
}

bool MapAnalysis::is_transit_cell(const int row, const int col) const noexcept {
    if (!is_walkable(row, col)) {
        return false;
    }

    if (degree(row, col) != 2) {
        return false;
    }

    bool north = is_walkable(row - 1, col);
    bool south = is_walkable(row + 1, col);
    bool east = is_walkable(row, col + 1);
    bool west = is_walkable(row, col - 1);

    return (north && south) || (east && west);
}

std::vector<Position> MapAnalysis::neighbors(const Position& pos) const {
    std::vector<Position> out;
    out.reserve(4);

    for (int k = 0; k < 4; ++k) {
        const Position nxt{pos.row + DR[k], pos.col + DC[k]};
        if (is_walkable(nxt.row, nxt.col)) {
            out.push_back(nxt);
        }
    }

    return out;
}

std::vector<Position> MapAnalysis::all_goal_cells_for(const char symbol) const {
    std::vector<Position> goals;

    for (int row = 0; row < level_.rows; ++row) {
        for (int col = 0; col < level_.cols; ++col) {
            if (level_.goal_at(row, col) == symbol) {
                goals.push_back(Position{row, col});
            }
        }
    }

    return goals;
}

std::vector<BoxRecord> MapAnalysis::collect_boxes(const State& state) const {
    std::vector<BoxRecord> boxes;

    for (int row = 0; row < state.rows; ++row) {
        for (int col = 0; col < state.cols; ++col) {
            const char box = state.box_at(row, col);
            if (box == '\0') {
                continue;
            }

            boxes.push_back(BoxRecord{
                box,
                Position{row, col},
                level_.goal_at(row, col) == box,
                is_chokepoint(row, col),
                degree(row, col)
            });
        }
    }

    return boxes;
}

std::vector<Position> MapAnalysis::find_relocation_candidates(const State& state, const Position& from) const {
    std::vector<Position> candidates;
    std::queue<Position> frontier;
    std::unordered_set<int> visited;

    frontier.push(from);
    visited.insert(index(from.row, from.col));

    while (!frontier.empty()) {
        const Position current = frontier.front();
        frontier.pop();

        for (const Position& nxt : neighbors(current)) {
            const int flat = index(nxt.row, nxt.col);
            if (visited.contains(flat)) {
                continue;
            }
            visited.insert(flat);

            if (state.has_box(nxt.row, nxt.col)) {
                continue;
            }

            frontier.push(nxt);

            if (level_.goal_at(nxt.row, nxt.col) != '\0') {
                continue;
            }
            if (degree(nxt.row, nxt.col) <= 2) {
                continue;
            }

            candidates.push_back(nxt);
        }
    }

    std::sort(candidates.begin(), candidates.end(), [&](const Position& a, const Position& b) {
        const int dist_a = std::abs(a.row - from.row) + std::abs(a.col - from.col);
        const int dist_b = std::abs(b.row - from.row) + std::abs(b.col - from.col);
        if (dist_a != dist_b) {
            return dist_a < dist_b;
        }
        return degree(a.row, a.col) > degree(b.row, b.col);
    });

    return candidates;
}

int MapAnalysis::index(const int row, const int col) const noexcept {
    return level_.index(row, col);
}
