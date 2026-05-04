#include "plan/ReservationTable.hpp"
#include "actions/Action.hpp"
#include "actions/ActionSemantics.hpp"

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

bool ReservationTable::is_cell_reserved(int row, int col, int time, int agent) const {
    auto it = cell_reservations_.find(CellReservation{row, col, time});
    return it != cell_reservations_.end() && it->second != agent;
}

bool ReservationTable::is_edge_reserved(const Position& from,
                                                 const Position& to,
                                                 int time,
                                                 int agent) const {
    auto it = edge_reservations_.find(EdgeReservation{from, to, time});
    return it != edge_reservations_.end() && it->second != agent;
}

bool ReservationTable::is_incoming_reserved(const Position& to, int time, int agent) const {
    for (const auto& [edge, owner] : edge_reservations_) {
        if (owner == agent) continue;
        if (edge.time == time && edge.to == to) return true;
    }
    return false;
}

void ReservationTable::reserve_cell(int row, int col, int time, int agent) {
    cell_reservations_[CellReservation{row, col, time}] = agent;
}

void ReservationTable::reserve_edge(const Position& from,
                                    const Position& to,
                                    int time,
                                    int agent) {
    edge_reservations_[EdgeReservation{from, to, time}] = agent;
}

void ReservationTable::reserve_incoming(const Position& to, int time, int agent) {
    reserve_edge(to, to, time, agent);
}

void ReservationTable::reserve_path(const std::vector<Action>& path, Position initial_pos, int agent, int start_time) {
    Position current_pos = initial_pos;

    // Reserve the start cell at time 0
    reserve_cell(current_pos.row, current_pos.col, start_time, agent);

    for (int t = 0; t < static_cast<int>(path.size()); ++t) {
        Position next_pos = ActionSemantics::compute_effect(current_pos, path[t]).agent_to;
        const int absolute_time = start_time + t;

        // Reserve traversal from time t to t+1
        reserve_edge(current_pos, next_pos, absolute_time, agent);

        // Reserve destination cell at arrival time t+1
        reserve_cell(next_pos.row, next_pos.col, absolute_time + 1, agent);
        reserve_incoming(next_pos, absolute_time, agent);

        current_pos = next_pos;
    }

    // Keep the final position occupied for some extra horizon.
    // This is a simple first approximation to prevent later agents from
    // walking through an agent that has already reached its destination.
    const Position goal = current_pos;
    const int final_time = start_time + static_cast<int>(path.size()) - 1;

    for (int t = final_time + 1; t <= final_time + 20; ++t) {
        reserve_cell(goal.row, goal.col, t, agent);
    }
}
