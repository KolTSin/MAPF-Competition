#pragma once

#include "domain/Level.hpp"
#include "hospital/HospitalTask.hpp"
#include "hospital/MapAnalysis.hpp"
#include "state/State.hpp"

#include <vector>

class HospitalTaskDecomposer {
public:
    std::vector<HospitalTask> decompose(
        const Level& level,
        const State& state,
        const MapAnalysis& analysis) const;

private:
    [[nodiscard]] static int manhattan(const Position& a, const Position& b) noexcept;
    [[nodiscard]] static int pick_agent_for_box(const Level& level, const State& state, char box, const Position& box_pos);
};
