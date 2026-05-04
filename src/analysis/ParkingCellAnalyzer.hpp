#pragma once

#include "analysis/LevelAnalysis.hpp"
#include "domain/Level.hpp"
#include "state/State.hpp"
#include <vector>

class ParkingCellAnalyzer {
public:
    [[nodiscard]] std::vector<Position> find_parking_cells(const Level& level, const State& state, const LevelAnalysis& analysis) const;
    [[nodiscard]] int score_parking_cell(Position p, const Level& level, const State& state, const LevelAnalysis& analysis) const;
};
