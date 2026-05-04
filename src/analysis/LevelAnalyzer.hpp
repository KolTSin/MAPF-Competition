#pragma once
#include "analysis/LevelAnalysis.hpp"
#include "domain/Level.hpp"
#include "state/State.hpp"

class LevelAnalyzer {
public:
    [[nodiscard]] LevelAnalysis analyze(const Level& level, const State& state) const;
};
