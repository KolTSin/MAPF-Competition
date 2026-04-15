#include "plan/Conflicts.hpp"
#include "domain/Position.hpp"
#include "plan/ConstraintTable.hpp"


void ConstraintTable::addConstraint(const Constraint& c){
    return;
}
bool ConstraintTable::forbidsVertex(int agent, Position p, int t) const{
    return true;
}
bool ConstraintTable::forbidsEdge(int agent, Position from, Position to, int t) const{
    return true;
}
bool ConstraintTable::forbidsFollow(int agent, Position to, int t) const{
    return true;
}