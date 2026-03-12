#pragma once

#include "actions/Action.hpp"
#include "domain/Level.hpp"
#include "state/State.hpp"

class ActionApplicator {
public:
    static bool is_applicable(const Level& level,
                              const State& state,
                              int agent_id,
                              const Action& action);

    static State apply(const Level& level,
                       const State& state,
                       int agent_id,
                       const Action& action);

private:
    static bool cell_has_other_agent(const State& state,
                                     int acting_agent,
                                     int row,
                                     int col);

    static bool cell_is_free(const Level& level,
                             const State& state,
                             int acting_agent,
                             int row,
                             int col);
};