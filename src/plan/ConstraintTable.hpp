#pragma once

#include "plan/Conflicts.hpp"
#include "domain/Position.hpp"

#include <vector>

class ConstraintTable {
public:
    void addConstraint(const Constraint& c);
    bool forbidsVertex(int agent, Position p, int t) const;
    bool forbidsEdge(int agent, Position from, Position to, int t) const;
    bool forbidsFollow(int agent, Position to, int t) const;

private:
    std::vector<Constraint> constraints_;
};
