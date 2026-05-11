#pragma once

#include "domain/Level.hpp"
#include "plan/Plan.hpp"
#include "state/State.hpp"

#include <string>
#include <unordered_map>
#include <unordered_set>

struct WaveProgressDecision {
    bool accept{true};
    std::string reason;
    State after;
    int goals_after{0};
    bool moved_any_box{false};
    bool completed_new_goal{false};
};

// Stateful filter for competitive HTN waves.  The scheduler may produce a wave
// that is locally executable and moves a box, but still only undoes a previous
// movement or returns the world to a previously accepted layout.  This guard
// simulates candidate waves before they are appended to the output plan and
// rejects cyclic/stagnating waves with a diagnostic reason.
class WaveProgressGuard {
public:
    void reset(const Level& level, const State& initial_state);

    [[nodiscard]] WaveProgressDecision assess(const Level& level,
                                              const State& before,
                                              const Plan& wave,
                                              int best_goals_completed);

    void remember_accepted(const State& state);

    [[nodiscard]] static int goals_completed(const Level& level, const State& state);
    [[nodiscard]] static std::string world_signature(const State& state);

private:
    std::unordered_set<std::string> seen_worlds_;
    std::unordered_map<std::string, int> trajectory_best_distance_;

    [[nodiscard]] static int nearest_goal_distance(const Level& level, char box, const State& state);
};
