#pragma once

#include "plan/Conflicts.hpp"
#include "domain/Position.hpp"
#include "plan/ReservationTable.hpp"

class ConflictTable: public ReservationTable {
public:
    void register_conflict(CellReservation res1, CellReservation res2);

private:
    std::unordered_map<Conflict, int, ConflictHasher> conflicts_;
}; 