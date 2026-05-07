#pragma once
#include "analysis/LevelAnalysis.hpp"
#include "domain/Level.hpp"
#include "state/State.hpp"

// Builds topological annotations for a level from walls, goals, and current state.
class LevelAnalyzer {
public:
    [[nodiscard]] LevelAnalysis analyze(const Level& level, const State& state) const;
};
