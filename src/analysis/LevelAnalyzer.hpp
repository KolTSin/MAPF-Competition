#pragma once
#include "analysis/LevelAnalysis.hpp"
#include "domain/Level.hpp"
#include "state/State.hpp"

// Converts a raw level grid into reusable topology facts.
//
// Input:
//   - Level: static map data such as bounds, walls, and goal symbols.
//   - State: current dynamic occupancy, forwarded to ParkingCellAnalyzer so
//     already occupied cells are not suggested as temporary storage.
//
// Output:
//   - LevelAnalysis: one CellInfo entry per grid coordinate plus convenience
//     lists for common categories such as free cells, corridors, rooms,
//     chokepoints, and parking cells.
//
// The analyzer intentionally treats walls as permanent obstacles and ignores
// agents/boxes for most topology labels. Dynamic occupancy only matters when
// calculating parking recommendations at the end of analyze().
class LevelAnalyzer {
public:
    [[nodiscard]] LevelAnalysis analyze(const Level& level, const State& state) const;
};
