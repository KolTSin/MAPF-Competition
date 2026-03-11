#pragma once

#include "actions/Action.hpp"
#include <string>
#include <vector>

struct JointAction {
    std::vector<Action> actions;

    [[nodiscard]] std::string to_string() const {
        std::string out;
        for (std::size_t i = 0; i < actions.size(); ++i) {
            if (i > 0) {
                out += "|";
            }
            out += actions[i].command;
        }
        return out;
    }
};