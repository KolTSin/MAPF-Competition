#pragma once

#include "domain/Position.hpp"

#include <vector>

// Per-cell annotations computed once from static walls/goals and then reused by
// task generation, parking selection, and the competitive solver.
struct CellInfo {
    bool is_free_static{false};     // Traversable if boxes/agents are ignored.
    bool is_corridor{false};        // Degree-two passage where blocking is costly.
    bool is_dead_end{false};        // Degree-one cell, useful for parking heuristics.
    bool is_room{false};            // Open area with more local maneuvering space.
    bool is_chokepoint{false};      // Narrow connector between larger regions.
    bool is_articulation{false};    // Removing this cell disconnects the map graph.
    bool is_goal_cell{false};       // Any goal symbol is present here.
    bool is_box_goal_cell{false};   // Goal is a lowercase box target.
    bool is_agent_goal_cell{false}; // Goal is a digit target for an agent.
    int component_id{-1};           // Connected-component id over free cells.
    int degree{0};                  // Number of static free neighbors.
    int parking_score{0};           // Higher means safer/better temporary storage.
};

// Compact grid-shaped analysis result. Position lists duplicate selected cells
// so callers can iterate a category without scanning the whole map.
struct LevelAnalysis {
    int rows{0};
    int cols{0};
    std::vector<CellInfo> cells;
    std::vector<Position> free_cells;
    std::vector<Position> corridor_cells;
    std::vector<Position> room_cells;
    std::vector<Position> chokepoints;
    std::vector<Position> parking_cells;

    [[nodiscard]] int index(Position p) const noexcept;
    [[nodiscard]] const CellInfo& at(Position p) const;
    CellInfo& at(Position p);
};
