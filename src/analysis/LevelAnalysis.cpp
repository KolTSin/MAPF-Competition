#include "analysis/LevelAnalysis.hpp"
#include <stdexcept>
int LevelAnalysis::index(Position p) const noexcept { return p.row * cols + p.col; }
const CellInfo& LevelAnalysis::at(Position p) const { if (p.row < 0 || p.row >= rows || p.col < 0 || p.col >= cols) throw std::out_of_range("LevelAnalysis::at const out of bounds"); return cells[index(p)]; }
CellInfo& LevelAnalysis::at(Position p) { if (p.row < 0 || p.row >= rows || p.col < 0 || p.col >= cols) throw std::out_of_range("LevelAnalysis::at out of bounds"); return cells[index(p)]; }
