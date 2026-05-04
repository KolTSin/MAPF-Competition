#include "analysis/ParkingCellAnalyzer.hpp"
#include <algorithm>

namespace { constexpr int DR[4] = {-1, 1, 0, 0}; constexpr int DC[4] = {0, 0, -1, 1}; }

int ParkingCellAnalyzer::score_parking_cell(Position p, const Level& level, const State& state, const LevelAnalysis& analysis) const {
    if (!level.in_bounds(p.row, p.col) || level.is_wall(p.row, p.col)) return -100000;
    int score = 0; const CellInfo& cell = analysis.at(p);
    if (state.has_box(p.row, p.col)) return -100000;
    if (cell.is_goal_cell) return -100000;
    score += 50; if (cell.is_room) score += 30;
    if (cell.is_articulation) score -= 50000;
    if (cell.is_chokepoint) score -= 10000;
    if (cell.is_corridor) score -= 1000;
    if (cell.is_dead_end) score -= 200;
    bool adjacent_chokepoint = false; bool adjacent_goal = false;
    for (int i = 0; i < 4; ++i) {
        Position n{p.row + DR[i], p.col + DC[i]};
        if (!level.in_bounds(n.row, n.col) || level.is_wall(n.row, n.col)) continue;
        const CellInfo& neighbor = analysis.at(n);
        adjacent_chokepoint = adjacent_chokepoint || neighbor.is_chokepoint;
        adjacent_goal = adjacent_goal || neighbor.is_goal_cell;
    }
    if (!adjacent_chokepoint) score += 20;
    if (!adjacent_goal) score += 10;
    return score;
}

std::vector<Position> ParkingCellAnalyzer::find_parking_cells(const Level& level, const State& state, const LevelAnalysis& analysis) const {
    std::vector<std::pair<Position, int>> scored; scored.reserve(analysis.free_cells.size());
    for (const Position p : analysis.free_cells) {
        const int score = score_parking_cell(p, level, state, analysis);
        if (score > 0) scored.emplace_back(p, score);
    }
    std::sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) { return a.second > b.second; });
    std::vector<Position> result; result.reserve(scored.size());
    for (const auto& [p, _] : scored) result.push_back(p);
    return result;
}
