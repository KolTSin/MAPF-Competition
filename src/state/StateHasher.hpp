#pragma once

#include "state/State.hpp"

#include <cstddef>

struct StateHasher {
    std::size_t operator()(const State& s) const noexcept {
        std::size_t hash = 0;

        auto hash_combine = [](std::size_t& seed, std::size_t value) {
            seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        };

        // Hash agent positions
        for (const auto& pos : s.agent_positions) {
            std::size_t h = std::hash<int>()(pos.row) ^
                            (std::hash<int>()(pos.col) << 1);
            hash_combine(hash, h);
        }

        // Hash box positions
        for (char b : s.box_pos) {
            hash_combine(hash, std::hash<char>()(b));
        }

        return hash;
    }
};