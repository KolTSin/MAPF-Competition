#include "plan/ConstraintTable.hpp"

void ConstraintTable::addConstraint(const Constraint& c) {
    constraints_.push_back(c);
}

bool ConstraintTable::forbidsVertex(int agent, Position p, int t) const {
    for (const Constraint& c : constraints_) {
        if (c.agent_id == agent && c.type == ConflictType::AgentVertex &&
            c.time == t && c.cell == p) {
            return true;
        }
    }
    return false;
}

bool ConstraintTable::forbidsEdge(int agent, Position from, Position to, int t) const {
    for (const Constraint& c : constraints_) {
        if (c.agent_id == agent && c.type == ConflictType::AgentEdgeSwap &&
            c.time == t && c.from[0] == from && c.to[0] == to) {
            return true;
        }
    }
    return false;
}

bool ConstraintTable::forbidsFollow(int agent, Position to, int t) const {
    for (const Constraint& c : constraints_) {
        if (c.agent_id == agent && c.type == ConflictType::AgentFollow &&
            c.time == t && c.cell == to) {
            return true;
        }
    }
    return false;
}
