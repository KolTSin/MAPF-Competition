#pragma once

#include "analysis/LevelAnalysis.hpp"
#include "domain/Level.hpp"
#include "state/State.hpp"
#include <vector>

// Ranks cells that are safe places to temporarily move an agent or blocking box.
//
// Input:
//   - Level: bounds and static walls used to reject impossible coordinates.
//   - State: current box occupancy used to reject already occupied cells.
//   - LevelAnalysis: topology labels produced by LevelAnalyzer, such as rooms,
//     corridors, chokepoints, and goal cells.
//
// Output:
//   - score_parking_cell(): a single heuristic score. Positive values mean the
//     cell is a useful parking candidate; very negative values mean it should
//     never be selected.
//   - find_parking_cells(): all positive-scoring candidates sorted from best to
//     worst, so callers can try safer temporary placements first.
class ParkingCellAnalyzer {
public:
    [[nodiscard]] std::vector<Position> find_parking_cells(const Level& level, const State& state, const LevelAnalysis& analysis) const;
    [[nodiscard]] int score_parking_cell(Position p, const Level& level, const State& state, const LevelAnalysis& analysis) const;
};
