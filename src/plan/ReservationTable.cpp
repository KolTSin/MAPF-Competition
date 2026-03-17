#include "plan/ReservationTable.hpp"

namespace {

inline void hash_combine(std::size_t& seed, std::size_t value) noexcept {
    seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

} // namespace

std::size_t CellReservationHasher::operator()(const CellReservation& r) const noexcept {
    std::size_t seed = 0;
    hash_combine(seed, std::hash<int>{}(r.row));
    hash_combine(seed, std::hash<int>{}(r.col));
    hash_combine(seed, std::hash<int>{}(r.time));
    return seed;
}

std::size_t EdgeReservationHasher::operator()(const EdgeReservation& r) const noexcept {
    std::size_t seed = 0;
    hash_combine(seed, std::hash<int>{}(r.from.row));
    hash_combine(seed, std::hash<int>{}(r.from.col));
    hash_combine(seed, std::hash<int>{}(r.to.row));
    hash_combine(seed, std::hash<int>{}(r.to.col));
    hash_combine(seed, std::hash<int>{}(r.time));
    return seed;
}

bool ReservationTable::is_cell_reserved(int row, int col, int time) const {
    return cell_reservations_.contains(CellReservation{row, col, time});
}

bool ReservationTable::is_edge_reserved(const Position& from,
                                        const Position& to,
                                        int time) const {
    return edge_reservations_.contains(EdgeReservation{from, to, time});
}

void ReservationTable::reserve_cell(int row, int col, int time) {
    cell_reservations_.insert(CellReservation{row, col, time});
}

void ReservationTable::reserve_edge(const Position& from,
                                    const Position& to,
                                    int time) {
    edge_reservations_.insert(EdgeReservation{from, to, time});
}

void ReservationTable::reserve_path(const std::vector<Position>& path) {
    if (path.empty()) {
        return;
    }

    for (int t = 0; t < static_cast<int>(path.size()); ++t) {
        reserve_cell(path[t].row, path[t].col, t);

        if (t > 0) {
            reserve_edge(path[t - 1], path[t], t - 1);
        }
    }

    // Keep the final position occupied for some extra horizon.
    // This is a simple first approximation to prevent later agents from
    // walking through an agent that has already reached its destination.
    const Position goal = path.back();
    const int final_time = static_cast<int>(path.size()) - 1;

    for (int t = final_time + 1; t <= final_time + 20; ++t) {
        reserve_cell(goal.row, goal.col, t);
    }
}