#pragma once

#include "domain/Level.hpp"
#include "domain/Position.hpp"
#include "state/State.hpp"

#include <unordered_set>
#include <vector>

struct BoxRecord {
    char symbol{'\0'};
    Position pos{};
    bool on_goal{false};
    bool in_chokepoint{false};
    int degree{0};
};

class MapAnalysis {
public:
    explicit MapAnalysis(const Level& level);

    [[nodiscard]] bool is_walkable(int row, int col) const noexcept;
    [[nodiscard]] int degree(int row, int col) const noexcept;
    [[nodiscard]] bool is_chokepoint(int row, int col) const noexcept;
    [[nodiscard]] bool is_transit_cell(int row, int col) const noexcept;

    [[nodiscard]] std::vector<Position> neighbors(const Position& pos) const;
    [[nodiscard]] std::vector<Position> all_goal_cells_for(char symbol) const;
    [[nodiscard]] std::vector<BoxRecord> collect_boxes(const State& state) const;
    [[nodiscard]] std::vector<Position> find_relocation_candidates(const State& state, const Position& from) const;

private:
    const Level& level_;
    std::vector<int> degree_map_;

    [[nodiscard]] int index(int row, int col) const noexcept;
};
