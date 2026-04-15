#pragma once

#include "domain/Position.hpp"
#include "actions/Action.hpp"

#include <vector>
#include <sstream>

struct AgentPlan {
    int agent{-1};
    std::vector<Action> actions;
    std::vector<Position> positions; // positions[t]

    void add(Action a, Position p){
        actions.push_back(a);
        positions.push_back(p);
    }

    bool valid() const {
        return !positions.empty() && positions.size() == actions.size() + 1;
    }

    int cost() const {
        return static_cast<int>(actions.size());
    }

    Position position_at(int t) const {
        if (positions.empty()) return {};
        if (t <= 0) return positions.front();
        if (t >= static_cast<int>(positions.size())) return positions.back();
        return positions[t];
    }

    std::string to_string() const {
        std::ostringstream os;
        for (int i = 0; i < static_cast<int>(positions.size()) - 1; i++){
            os << "|" << positions[i].to_string() << " -> " << actions[i].to_string() << " -> " << positions[i+1].to_string();
        }
        os << "EOM";
        return os.str();
    }

    bool operator==(const AgentPlan& b) const noexcept {
        return this -> positions == b.positions;
    }
};