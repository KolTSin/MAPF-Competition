#pragma once
#include "domain/Position.hpp"
#include <vector>
struct CellInfo { bool is_free_static{false}; bool is_corridor{false}; bool is_dead_end{false}; bool is_room{false}; bool is_chokepoint{false}; bool is_articulation{false}; bool is_goal_cell{false}; bool is_box_goal_cell{false}; bool is_agent_goal_cell{false}; int component_id{-1}; int degree{0}; int parking_score{0}; };
struct LevelAnalysis { int rows{0}; int cols{0}; std::vector<CellInfo> cells; std::vector<Position> free_cells; std::vector<Position> corridor_cells; std::vector<Position> room_cells; std::vector<Position> chokepoints; std::vector<Position> parking_cells; [[nodiscard]] int index(Position p) const noexcept; [[nodiscard]] const CellInfo& at(Position p) const; CellInfo& at(Position p); };
