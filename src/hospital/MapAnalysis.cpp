#include "hospital/MapAnalysis.hpp"

#include <algorithm>

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

    for (const Position& cell : neighbors(from)) {
        if (state.has_box(cell.row, cell.col)) {
            continue;
        }
        if (level_.goal_at(cell.row, cell.col) != '\0') {
            continue;
        }
        if (degree(cell.row, cell.col) <= 2) {
            continue;
        }
        candidates.push_back(cell);
    }

    std::sort(candidates.begin(), candidates.end(), [&](const Position& a, const Position& b) {
        return degree(a.row, a.col) > degree(b.row, b.col);
    });

    return candidates;
}

int MapAnalysis::index(const int row, const int col) const noexcept {
    return level_.index(row, col);
}
