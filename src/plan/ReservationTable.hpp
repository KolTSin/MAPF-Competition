#pragma once

#include "domain/Position.hpp"
#include "actions/Action.hpp"

#include <cstddef>
#include <optional>
#include <unordered_map>
#include <vector>

struct CellReservation {
    int row;
    int col;
    int time;

    bool operator==(const CellReservation& other) const noexcept {
        return row == other.row &&
               col == other.col &&
               time == other.time;
    }
};

struct EdgeReservation {
    Position from;
    Position to;
    int time;

    bool operator==(const EdgeReservation& other) const noexcept {
        return from == other.from &&
               to == other.to &&
               time == other.time;
    }
};

struct CellReservationHasher {
    std::size_t operator()(const CellReservation& r) const noexcept;
};

struct EdgeReservationHasher {
    std::size_t operator()(const EdgeReservation& r) const noexcept;
};

class ReservationTable {
public:
    void clear();
    [[nodiscard]] bool empty() const;
    [[nodiscard]] bool is_cell_reserved(int row, int col, int time, int agent) const;
    [[nodiscard]] bool is_edge_reserved(const Position& from,
                                        const Position& to,
                                        int time,
                                        int agent) const;
    [[nodiscard]] bool is_incoming_reserved(const Position& to,
                                            int time,
                                            int agent) const;
    [[nodiscard]] bool is_outgoing_reserved(const Position& from,
                                            int time,
                                            int agent) const;

    void reserve_cell(int row, int col, int time, int agent);
    void reserve_edge(const Position& from,
                      const Position& to,
                      int time,
                      int agent);
    void reserve_incoming(const Position& to, int time, int agent);

    void reserve_path(const std::vector<Action>& path, Position initial_pos, int agent, int start_time = 0);
    void reserve_agent_path(int agent_id, const std::vector<Position>& trajectory, int start_time);
    void reserve_box_path(char box_char, const std::vector<Position>& trajectory, int start_time, int persistence_horizon = 20);
    [[nodiscard]] bool can_occupy_agent(int agent_id, Position pos, int time) const;
    [[nodiscard]] bool can_move_agent(int agent_id, Position from, Position to, int time_from, int time_to) const;
    [[nodiscard]] bool can_occupy_box(char box_char, Position pos, int time) const;
    [[nodiscard]] bool can_move_box(char box_char, Position from, Position to, int time_from, int time_to) const;
    [[nodiscard]] bool can_apply_transition(int agent_id,
                                            Position agent_from,
                                            Position agent_to,
                                            std::optional<char> box_char,
                                            std::optional<Position> box_from,
                                            std::optional<Position> box_to,
                                            int time_from) const;

private:
    std::unordered_map<CellReservation, int, CellReservationHasher> cell_reservations_;
    std::unordered_map<EdgeReservation, int, EdgeReservationHasher> edge_reservations_;
};
