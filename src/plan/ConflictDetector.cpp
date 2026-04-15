#include "plan/ConflictDetector.hpp"
#include "plan/AgentPlan.hpp"
#include "plan/Conflicts.hpp"
#include "domain/Position.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>

Position ConflictDetector::getPositionAt(const AgentPlan& plan, int t) {
    // std::cerr << "agent pos:" << plan.agent << std::endl;
    assert(!plan.positions.empty());

    if (t <= 0) {
        return plan.positions.front();
    }

    const int last = static_cast<int>(plan.positions.size()) - 1;
    if (t >= last) {
        return plan.positions.back();
    }

    return plan.positions[t];
}

Conflict ConflictDetector::findFirstConflict(const std::vector<AgentPlan>& plans) {
    // for (AgentPlan a : plans){
    //     std::cerr << a.to_string() << std::endl;
    // }
    Conflict c_err;
    // std::cerr << c_err.agents[0] << std::endl;
    
    if (plans.empty()) {
        // std::cerr << "plans empty!" << std::endl;
        return c_err;
    }
    // std::cerr << "plans are not empty!" << std::endl;
    int makespan = 0;
    for (const auto& p : plans) {
        makespan = std::max(makespan, static_cast<int>(p.actions.size()));
    }
    std::cerr << "starting to look for conflicts" << std::endl;
    for (int t = 0; t < makespan; ++t) {
        // std::cerr << "time: " << t << std::endl;
        for (int i = 0; i < static_cast<int>(plans.size()); ++i) {
        // if (plans[i].positions.empty()){
        //     continue;
        // }
            for (int j = i + 1; j < static_cast<int>(plans.size()); ++j) {
                // if (plans[j].positions.empty()){
                //     continue;
                // }
                const Position src_i = getPositionAt(plans[i], t);
                const Position dst_i = getPositionAt(plans[i], t + 1);

                const Position src_j = getPositionAt(plans[j], t);
                const Position dst_j = getPositionAt(plans[j], t + 1);

                // 1) Vertex conflict: same destination cell after the step.
                if (dst_i == dst_j) {
                    Conflict c;
                    c.agents[0] = i;
                    c.agents[1] = j;
                    c.time   = t + 1;
                    c.cell   = dst_i;
                    std::cerr << "vertex conflicts found! @ " << dst_i.to_string() << std::endl;
                    return c;
                }

                // 2) Edge conflict / swap.
                if (src_i == dst_j && src_j == dst_i && src_i != src_j) {
                    Conflict c;
                    c.agents[0] = i;
                    c.agents[1] = j;
                    c.time   = t;
                    c.from[0]  = src_i;
                    c.to[0]    = dst_i;
                    c.from[1]  = src_j;
                    c.to[1]    = dst_j;
                    return c;
                }

                // 3) Follow conflict:
                // agent i moves into the cell that agent j occupied at the start of the step,
                // while j moves away elsewhere.
                if (dst_i == src_j && src_j != dst_j && !(src_i == dst_j && dst_i == src_j)) {
                    Conflict c;
                    c.agents[0] = i;      // follower
                    c.agents[1] = j;      // leader
                    c.time   = t;      // step time
                    c.cell   = src_j;  // leader source cell
                    std::cerr << "follow conflicts found!" << std::endl;
                    return c;
                }

                // Symmetric follow case.
                if (dst_j == src_i && src_i != dst_i && !(src_j == dst_i && dst_j == src_i)) {
                    Conflict c;
                    c.agents[0] = j;      // follower
                    c.agents[1] = i;      // leader
                    c.time   = t;      // step time
                    c.cell   = src_i;  // leader source cell
                    std::cerr << "symmetric follow conflicts found!" << std::endl;
                    return c;
                }
            }
        }
    }

    std::cerr << "no conflicts found!" << std::endl;
    return c_err;
}