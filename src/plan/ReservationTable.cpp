#include "plan/ReservationTable.hpp"
#include "actions/Action.hpp"
#include "actions/ActionSemantics.hpp"
#include "plan/AgentPlan.hpp"

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
    if (it == cell_reservations_.end()) return false;
    for (int owner : it->second) {
        if (owner != agent) return true;
    }
    return false;
}
void ReservationTable::clear() {
    cell_reservations_.clear();
    edge_reservations_.clear();
}

bool ReservationTable::empty() const {
    return cell_reservations_.empty() && edge_reservations_.empty();
}

bool ReservationTable::is_edge_reserved(const Position& from,
                                                 const Position& to,
                                                 int time,
                                                 int agent) const {
    auto it = edge_reservations_.find(EdgeReservation{from, to, time});
    if (it == edge_reservations_.end()) return false;
    for (int owner : it->second) {
        if (owner != agent) return true;
    }
    return false;
}

bool ReservationTable::is_incoming_reserved(const Position& to, int time, int agent) const {
    for (const auto& [edge, owners] : edge_reservations_) {
        if (edge.time != time || edge.to != to) continue;
        for (int owner : owners) {
            if (owner != agent) return true;
        }
    }
    return false;
}

bool ReservationTable::is_outgoing_reserved(const Position& from, int time, int agent) const {
    for (const auto& [edge, owners] : edge_reservations_) {
        if (edge.time != time || edge.from != from) continue;
        for (int owner : owners) {
            if (owner != agent) return true;
        }
    }
    return false;
}

void ReservationTable::reserve_cell(int row, int col, int time, int agent) {
    cell_reservations_[CellReservation{row, col, time}].push_back(agent);
}

void ReservationTable::reserve_edge(const Position& from,
                                    const Position& to,
                                    int time,
                                    int agent) {
    edge_reservations_[EdgeReservation{from, to, time}].push_back(agent);
}

void ReservationTable::reserve_incoming(const Position& to, int time, int agent) {
    reserve_edge(to, to, time, agent);
}

void ReservationTable::reserve_path(const AgentPlan& plan, int start_time) {
    if (plan.positions.empty()) return;

    const int agent = plan.agent;
    reserve_cell(plan.positions.front().row, plan.positions.front().col, start_time, agent);

    for (std::size_t i = 0; i < plan.actions.size(); ++i) {
        const Position current_pos = plan.positions[i];
        const Position next_pos = (i + 1 < plan.positions.size())
            ? plan.positions[i + 1]
            : ActionSemantics::compute_effect(current_pos, plan.actions[i]).agent_to;
        const int absolute_time = start_time + static_cast<int>(i);

        reserve_edge(current_pos, next_pos, absolute_time, agent);
        reserve_cell(next_pos.row, next_pos.col, absolute_time + 1, agent);
        reserve_incoming(next_pos, absolute_time, agent);
    }

    const Position goal = (plan.positions.size() == plan.actions.size() + 1)
        ? plan.positions.back()
        : plan.position_at(static_cast<int>(plan.actions.size()));
    const int final_time = start_time + static_cast<int>(plan.actions.size());

    for (int t = final_time + 1; t <= final_time + 20; ++t) {
        reserve_cell(goal.row, goal.col, t, agent);
    }
}

void ReservationTable::reserve_agent_path(int agent_id, const std::vector<Position>& trajectory, int start_time) {
    if (trajectory.empty()) return;
    reserve_cell(trajectory.front().row, trajectory.front().col, start_time, agent_id);
    for (std::size_t i = 1; i < trajectory.size(); ++i) {
        const int t = start_time + static_cast<int>(i) - 1;
        reserve_edge(trajectory[i - 1], trajectory[i], t, agent_id);
        reserve_cell(trajectory[i].row, trajectory[i].col, t + 1, agent_id);
        reserve_incoming(trajectory[i], t, agent_id);
    }
}

void ReservationTable::reserve_box_path(char box_char, const std::vector<Position>& trajectory, int start_time, int persistence_horizon) {
    if (trajectory.empty()) return;

    const int box_owner = -1000 - static_cast<int>(box_char);
    reserve_agent_path(box_owner, trajectory, start_time);

    const Position final_cell = trajectory.back();
    const int final_time = start_time + static_cast<int>(trajectory.size()) - 1;
    for (int t = final_time + 1; t <= final_time + persistence_horizon; ++t) {
        reserve_cell(final_cell.row, final_cell.col, t, box_owner);
    }
}

bool ReservationTable::can_occupy_agent(int agent_id, Position pos, int time) const {
    return !is_cell_reserved(pos.row, pos.col, time, agent_id);
}
bool ReservationTable::can_move_agent(int agent_id, Position from, Position to, int time_from, int) const {
    return !is_edge_reserved(to, from, time_from, agent_id) &&
           !is_cell_reserved(to.row, to.col, time_from + 1, agent_id) &&
           !is_incoming_reserved(to, time_from, agent_id) &&
           !is_outgoing_reserved(from, time_from, agent_id) &&
           !is_outgoing_reserved(to, time_from, agent_id);
}
bool ReservationTable::can_occupy_box(char box_char, Position pos, int time) const {
    return can_occupy_agent(-1000 - static_cast<int>(box_char), pos, time);
}
bool ReservationTable::can_move_box(char box_char, Position from, Position to, int time_from, int time_to) const {
    return can_move_agent(-1000 - static_cast<int>(box_char), from, to, time_from, time_to);
}
bool ReservationTable::can_apply_transition(int agent_id,
                                            Position agent_from,
                                            Position agent_to,
                                            std::optional<char> box_char,
                                            std::optional<Position> box_from,
                                            std::optional<Position> box_to,
                                            int time_from) const {
    if (!can_move_agent(agent_id, agent_from, agent_to, time_from, time_from + 1)) return false;
    if (box_char && box_from && box_to) {
        if (!can_move_box(*box_char, *box_from, *box_to, time_from, time_from + 1)) return false;
    }
    return true;
}
