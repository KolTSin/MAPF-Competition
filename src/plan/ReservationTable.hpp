#pragma once

#include "domain/Position.hpp"

#include <cstddef>
#include <unordered_set>
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
    [[nodiscard]] bool is_cell_reserved(int row, int col, int time) const;
    [[nodiscard]] bool is_edge_reserved(const Position& from,
                                        const Position& to,
                                        int time) const;

    void reserve_cell(int row, int col, int time);
    void reserve_edge(const Position& from,
                      const Position& to,
                      int time);

    void reserve_path(const std::vector<Position>& path);

private:
    std::unordered_set<CellReservation, CellReservationHasher> cell_reservations_;
    std::unordered_set<EdgeReservation, EdgeReservationHasher> edge_reservations_;
};