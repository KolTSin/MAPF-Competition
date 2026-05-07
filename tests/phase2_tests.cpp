#include "tasks/TaskGenerator.hpp"
#include "tasks/Task.hpp"
#include "tasks/TaskScheduler.hpp"
#include "tasks/DependencyBuilder.hpp"
#include "tasks/TaskPrioritizer.hpp"
#include "analysis/LevelAnalyzer.hpp"
#include "actions/ActionApplicator.hpp"
#include "actions/ActionSemantics.hpp"
#include "hospital/AgentPathPlanner.hpp"
#include "hospital/BlockerResolver.hpp"
#include "hospital/BoxTransportPlanner.hpp"
#include "hospital/LocalRepair.hpp"
#include "plan/ConflictDetector.hpp"
#include "plan/ReservationTable.hpp"
#include "solvers/CompetitiveSolver.hpp"
#include "search/heuristics/Heuristic.hpp"
#include "parser/LevelParser.hpp"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

static Level make_level() {
    Level l;
    l.rows = 5; l.cols = 5;
    l.walls.assign(25, false);
    l.goals.assign(25, '\0');
    l.agent_colors.fill(Color::Unknown);
    l.box_colors.fill(Color::Unknown);
    l.agent_colors[0] = Color::Blue;
    l.agent_colors[1] = Color::Red;
    l.box_colors['A' - 'A'] = Color::Blue;
    l.box_colors['B' - 'A'] = Color::Red;
    l.goals[l.index(1,3)] = 'A';
    l.goals[l.index(3,3)] = 'B';
    return l;
}

static State make_state() {
    State s;
    s.rows = 5; s.cols = 5;
    s.agent_positions = {Position{1,1}, Position{4,4}};
    s.box_pos.assign(25, '\0');
    s.set_box(1,2,'A');
    s.set_box(2,2,'B');
    return s;
}


struct PlanValidationResult {
    bool valid{true};
    std::string reason;
    State final_state;
};

static bool all_goals_satisfied(const Level& level, const State& state, std::string* reason) {
    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const char goal = level.goal_at(r, c);
            if (goal >= 'A' && goal <= 'Z') {
                if (state.box_at(r, c) != goal) {
                    if (reason) {
                        std::ostringstream out;
                        out << "box_goal_unsatisfied " << goal << " at (" << r << "," << c << ")";
                        *reason = out.str();
                    }
                    return false;
                }
            } else if (goal >= '0' && goal <= '9') {
                const int agent_id = goal - '0';
                if (agent_id >= state.num_agents() || !(state.agent_positions[agent_id] == Position{r, c})) {
                    if (reason) {
                        std::ostringstream out;
                        out << "agent_goal_unsatisfied " << goal << " at (" << r << "," << c << ")";
                        *reason = out.str();
                    }
                    return false;
                }
            }
        }
    }
    return true;
}

static PlanValidationResult validate_plan_solves(const Level& level,
                                                 const State& initial_state,
                                                 const Plan& plan) {
    PlanValidationResult result;
    result.final_state = initial_state;

    if (plan.empty()) {
        result.valid = false;
        result.reason = "empty_plan";
        return result;
    }

    std::string conflict_reason;
    if (ConflictDetector::has_conflict(plan, initial_state, &conflict_reason)) {
        result.valid = false;
        result.reason = "conflict: " + conflict_reason;
        return result;
    }

    State current = initial_state;
    for (std::size_t t = 0; t < plan.steps.size(); ++t) {
        const JointAction& step = plan.steps[t];
        if (step.actions.size() != static_cast<std::size_t>(initial_state.num_agents())) {
            std::ostringstream out;
            out << "wrong_joint_action_arity at t=" << t << " expected=" << initial_state.num_agents()
                << " actual=" << step.actions.size();
            result.valid = false;
            result.reason = out.str();
            return result;
        }

        std::vector<ActionEffect> effects;
        effects.reserve(step.actions.size());
        for (int agent_id = 0; agent_id < current.num_agents(); ++agent_id) {
            const Action& action = step.actions[static_cast<std::size_t>(agent_id)];
            if (!ActionApplicator::is_applicable(level, current, agent_id, action)) {
                std::ostringstream out;
                out << "inapplicable_action at t=" << t << " agent=" << agent_id
                    << " action=" << action.to_string();
                result.valid = false;
                result.reason = out.str();
                return result;
            }
            effects.push_back(ActionSemantics::compute_effect(current.agent_positions[agent_id], action));
        }

        State next = current;
        for (int agent_id = 0; agent_id < current.num_agents(); ++agent_id) {
            const ActionEffect& effect = effects[static_cast<std::size_t>(agent_id)];
            if (effect.moves_box) {
                const char moved = current.box_at(effect.box_from.row, effect.box_from.col);
                next.set_box(effect.box_from.row, effect.box_from.col, '\0');
                next.set_box(effect.box_to.row, effect.box_to.col, moved);
            }
            next.agent_positions[agent_id] = effect.agent_to;
        }
        current = std::move(next);
    }

    std::string goal_reason;
    if (!all_goals_satisfied(level, current, &goal_reason)) {
        result.valid = false;
        result.reason = goal_reason;
        result.final_state = current;
        return result;
    }

    result.final_state = current;
    return result;
}

class ConstantHeuristic : public IHeuristic {
public:
    int evaluate(const State&) const override { return 0; }
    std::string name() const override { return "constant"; }
};

int main() {
    {
        Level l;
        l.rows = 4; l.cols = 4;
        l.walls.assign(16, false);
        l.goals.assign(16, '\0');
        l.agent_colors.fill(Color::Unknown);
        l.box_colors.fill(Color::Unknown);
        l.agent_colors[0] = Color::Blue;
        l.box_colors['A' - 'A'] = Color::Blue;

        State s;
        s.rows = 4; s.cols = 4;
        s.agent_positions = {Position{0,0}};
        s.box_pos.assign(16, '\0');
        s.set_box(1,1,'A');

        Task deliver;
        deliver.task_id = 0;
        deliver.type = TaskType::DeliverBoxToGoal;
        deliver.agent_id = 0;
        deliver.box_id = 'A';
        deliver.box_pos = Position{1,1};
        deliver.goal_pos = Position{1,3};

        Task blocker;
        blocker.task_id = 1;
        blocker.type = TaskType::MoveBlockingBoxToParking;
        blocker.agent_id = 0;
        blocker.box_id = 'A';
        blocker.box_pos = Position{1,1};
        blocker.parking_pos = Position{2,1};
        blocker.goal_pos = blocker.parking_pos;

        TaskPrioritizer prio;
        std::vector<Task> tasks = {deliver, blocker};
        prio.score(l, s, tasks);

        // deliver: max(0,30-2)+20-(2+2) = 44
        assert(tasks[0].priority == 44);
        // blocker: 25+30-10-1 = 44
        assert(tasks[1].priority == 44);
    }

    {
        Level l;
        l.rows = 5; l.cols = 5;
        l.walls.assign(25, false);
        l.goals.assign(25, '\0');
        l.agent_colors.fill(Color::Unknown);
        l.box_colors.fill(Color::Unknown);
        l.agent_colors[0] = Color::Blue;
        l.box_colors['A' - 'A'] = Color::Blue;

        State s;
        s.rows = 5; s.cols = 5;
        s.agent_positions = {Position{2,1}};
        s.box_pos.assign(25, '\0');
        s.set_box(2,2,'A');

        // Simulate corridor unblocking first, then goal delivery for the same box.
        Task unblock;
        unblock.task_id = 0;
        unblock.type = TaskType::MoveBlockingBoxToParking;
        unblock.agent_id = 0;
        unblock.box_id = 'A';
        unblock.box_pos = Position{2,2};
        unblock.parking_pos = Position{1,2};
        unblock.goal_pos = unblock.parking_pos;
        unblock.priority = 50;

        Task deliver;
        deliver.task_id = 1;
        deliver.type = TaskType::DeliverBoxToGoal;
        deliver.agent_id = 0;
        deliver.box_id = 'A';
        deliver.box_pos = Position{1,2};
        deliver.goal_pos = Position{2,4};
        deliver.priority = 40;

        DependencyBuilder deps;
        DependencyGraph graph = deps.build_graph({unblock, deliver});
        // Unblock must happen before delivery of the same box.
        assert(graph.predecessors[deliver.task_id].size() == 1);
        assert(graph.predecessors[deliver.task_id][0] == unblock.task_id);

        BoxTransportPlanner planner;
        TaskPlan p0 = planner.plan(l, s, unblock);
        assert(p0.success);
        State after_unblock = s;
        for (const Action& a : p0.primitive_actions) {
            after_unblock = ActionApplicator::apply(l, after_unblock, 0, a);
        }
        assert(after_unblock.box_at(1,2) == 'A');

        Task deliver_after = deliver;
        deliver_after.box_pos = Position{1,2};
        TaskPlan p1 = planner.plan(l, after_unblock, deliver_after);
        assert(p1.success);
        State final_state = after_unblock;
        for (const Action& a : p1.primitive_actions) {
            final_state = ActionApplicator::apply(l, final_state, 0, a);
        }
        assert(final_state.box_at(2,4) == 'A');
    }

    {
        ReservationTable rt;
        Position a{1,1}, b{1,2};
        rt.reserve_edge(a,b,0,0);
        assert(rt.is_incoming_reserved(b,0,1));
    }

    {
        ReservationTable rt;
        rt.reserve_box_path('B', {Position{1,2}}, 0, 20);
        assert(!rt.can_occupy_agent(0, Position{1,2}, 10));
        assert(!rt.can_occupy_box('A', Position{1,2}, 10));
        assert(rt.can_occupy_box('B', Position{1,2}, 10));
    }

    {
        Level l;
        l.rows = 3; l.cols = 5;
        l.walls.assign(15, false);
        l.goals.assign(15, '\0');
        l.agent_colors.fill(Color::Unknown);
        l.box_colors.fill(Color::Unknown);
        l.agent_colors[0] = Color::Blue;
        l.box_colors['A' - 'A'] = Color::Blue;

        State s;
        s.rows = 3; s.cols = 5;
        s.agent_positions = {Position{1,1}};
        s.box_pos.assign(15, '\0');
        s.set_box(1,2,'A');

        Task push_to_reserved;
        push_to_reserved.task_id = 10;
        push_to_reserved.type = TaskType::DeliverBoxToGoal;
        push_to_reserved.agent_id = 0;
        push_to_reserved.box_id = 'A';
        push_to_reserved.box_pos = Position{1,2};
        push_to_reserved.goal_pos = Position{1,3};

        ReservationTable rt;
        for (int t = 1; t <= 5000; ++t) rt.reserve_cell(1, 3, t, -1000 - static_cast<int>('B'));

        BoxTransportPlanner planner;
        TaskPlan blocked = planner.plan(l, s, push_to_reserved, rt, 0);
        assert(!blocked.success);
        assert(blocked.failure_reason == "no_path_for_single_box");
    }

    {
        Level l;
        l.rows = 3; l.cols = 5;
        l.walls.assign(15, false);
        l.goals.assign(15, '\0');
        l.agent_colors.fill(Color::Unknown);
        l.box_colors.fill(Color::Unknown);

        State s;
        s.rows = 3; s.cols = 5;
        s.agent_positions = {Position{1,1}};
        s.box_pos.assign(15, '\0');

        Task walk_through_reserved;
        walk_through_reserved.task_id = 11;
        walk_through_reserved.type = TaskType::MoveAgentToGoal;
        walk_through_reserved.agent_id = 0;
        walk_through_reserved.goal_pos = Position{1,3};

        ReservationTable rt;
        rt.reserve_box_path('B', {Position{1,2}}, 0, 20);

        AgentPathPlanner planner;
        TaskPlan blocked = planner.plan(l, s, walk_through_reserved, rt);
        assert(!blocked.success);
        assert(blocked.failure_reason == "no_path_for_agent_reposition");
    }

    {
        LocalRepair lr;
        TaskPlan failed;
        failed.success = false;
        failed.failure_reason = "stage_ok_1";
        Task dummy;
        RepairResult repaired = lr.repair(make_level(), make_state(), dummy, failed);
        assert(repaired.plan.success);
        assert(lr.last_outcome() == RepairStageOutcome::AlternateAgent);
    }

    {
        Level l;
        l.rows = 3; l.cols = 6;
        l.walls.assign(18, false);
        l.goals.assign(18, '\0');
        l.agent_colors.fill(Color::Unknown);
        l.box_colors.fill(Color::Unknown);
        l.agent_colors[0] = Color::Blue;
        l.agent_colors[1] = Color::Blue;
        l.box_colors['A' - 'A'] = Color::Blue;
        l.goals[l.index(1,5)] = 'A';

        State s;
        s.rows = 3; s.cols = 6;
        s.agent_positions = {Position{1,1}, Position{0,0}};
        s.box_pos.assign(18, '\0');
        s.set_box(1,3,'A');

        Task t0; t0.task_id = 0; t0.type = TaskType::MoveAgentToGoal; t0.agent_id = 0; t0.goal_pos = Position{1,2}; t0.priority = 100;
        Task t1; t1.task_id = 1; t1.type = TaskType::MoveBlockingBoxToParking; t1.agent_id = 1; t1.box_id = 'A'; t1.box_pos = Position{1,3}; t1.parking_pos = Position{0,3}; t1.priority = 90;
        Task t2; t2.task_id = 2; t2.type = TaskType::DeliverBoxToGoal; t2.agent_id = 0; t2.box_id = 'A'; t2.box_pos = Position{1,3}; t2.goal_pos = Position{1,5}; t2.priority = 80;

        TaskScheduler sched;
        Plan p = sched.build_plan(l, s, {t0, t1, t2});
        if (!p.empty() && p.steps.size() > 2) {
            assert(p.steps[1].actions[0].type == ActionType::NoOp);
        }
    }

    {
        Task a; a.task_id = 1; a.type = TaskType::MoveBlockingBoxToParking; a.box_id = 'A';
        Task b; b.task_id = 2; b.type = TaskType::DeliverBoxToGoal; b.box_id = 'A'; b.agent_id = 0;
        Task c; c.task_id = 3; c.type = TaskType::MoveAgentToGoal; c.agent_id = 0;
        Task d; d.task_id = 4; d.type = TaskType::MoveAgentToGoal; d.agent_id = 1;

        DependencyBuilder builder;
        DependencyGraph graph = builder.build_graph({a,b,c,d});
        assert(graph.predecessors[2].size() >= 1);
        assert(graph.predecessors[3].size() >= 1);

        std::unordered_map<int, int> remaining;
        for (const auto& [task_id, preds] : graph.predecessors) remaining[task_id] = static_cast<int>(preds.size());
        auto newly_ready = graph.mark_completed_and_get_new_ready(1, remaining);
        assert(newly_ready.size() <= graph.successors[1].size());
        auto no_dupes = builder.build({a,b,c,d});
        assert(no_dupes[2].size() >= 1);
    }

    {
        Task access_blocker;
        access_blocker.task_id = 10;
        access_blocker.type = TaskType::MoveBlockingBoxToParking;
        access_blocker.box_id = 'B';
        access_blocker.box_pos = Position{1, 1};
        access_blocker.parking_pos = Position{1, 2};
        access_blocker.goal_pos = access_blocker.parking_pos;
        access_blocker.unblocks_box_id = 'A';
        access_blocker.debug_label = "diagnostic_only_without_for_suffix";

        Task deliver;
        deliver.task_id = 11;
        deliver.type = TaskType::DeliverBoxToGoal;
        deliver.box_id = 'A';
        deliver.box_pos = Position{3, 3};
        deliver.goal_pos = Position{3, 4};

        DependencyBuilder builder;
        DependencyGraph graph = builder.build_graph({deliver, access_blocker});
        assert(graph.predecessors[deliver.task_id].size() == 1);
        assert(graph.predecessors[deliver.task_id][0] == access_blocker.task_id);

        Task explicit_dependency;
        explicit_dependency.task_id = 12;
        explicit_dependency.type = TaskType::MoveAgentToGoal;
        explicit_dependency.box_pos = Position{5, 5};
        explicit_dependency.goal_pos = Position{5, 6};
        explicit_dependency.parking_pos = explicit_dependency.goal_pos;
        explicit_dependency.dependencies.push_back(deliver.task_id);
        graph = builder.build_graph({access_blocker, deliver, explicit_dependency});
        assert(graph.predecessors[explicit_dependency.task_id].size() == 1);
        assert(graph.predecessors[explicit_dependency.task_id][0] == deliver.task_id);
    }

    {
        Level l;
        l.rows = 3; l.cols = 6;
        l.walls.assign(18, false);
        l.goals.assign(18, '\0');
        l.agent_colors.fill(Color::Unknown);
        l.box_colors.fill(Color::Unknown);
        State s;
        s.rows = 3; s.cols = 6;
        s.agent_positions = {Position{1,1}, Position{1,4}};
        s.box_pos.assign(18, '\0');

        Task t0; t0.task_id = 0; t0.type = TaskType::MoveAgentToGoal; t0.agent_id = 0; t0.goal_pos = Position{1,2}; t0.priority = 10;
        Task t1; t1.task_id = 1; t1.type = TaskType::MoveAgentToGoal; t1.agent_id = 1; t1.goal_pos = Position{1,5}; t1.priority = 10;

        TaskScheduler sched;
        Plan p = sched.build_plan(l, s, {t0, t1});
        assert(p.steps.size() <= 20);
    }

    {
        const std::vector<std::string> ma_simple_levels{
            "levels/MAsimple1.lvl",
            "levels/MAsimple2.lvl",
            "levels/MAsimple3.lvl",
            "levels/MAsimple4.lvl",
            "levels/MAsimple5.lvl"
        };
        for (const std::string& level_path : ma_simple_levels) {
            std::string resolved = level_path;
            if (!std::filesystem::exists(resolved)) {
                resolved = "../" + level_path;
            }
            assert(std::filesystem::exists(resolved));
            std::ifstream in(resolved);
            assert(in.good());
            ParsedLevel parsed = LevelParser::parse(in);
            CompetitiveSolver solver;
            ConstantHeuristic h;
            Plan p = solver.solve(parsed.level, parsed.initial_state, h);
            PlanValidationResult validation = validate_plan_solves(parsed.level, parsed.initial_state, p);
            if (!validation.valid) {
                std::cerr << "MAsimple failure: " << validation.reason << " on " << level_path << "\n";
            }
            assert(validation.valid);
        }
    }

    {
        Level l;
        l.rows = 3; l.cols = 5;
        l.walls.assign(15, false);
        l.goals.assign(15, '\0');
        l.agent_colors.fill(Color::Unknown);
        l.box_colors.fill(Color::Unknown);
        l.agent_colors[0] = Color::Blue;
        l.box_colors['A' - 'A'] = Color::Blue;
        l.goals[l.index(1,4)] = 'A';

        State s;
        s.rows = 3; s.cols = 5;
        s.agent_positions = {Position{1,1}};
        s.box_pos.assign(15, '\0');
        s.set_box(1,2,'A');

        CompetitiveSolver solver;
        ConstantHeuristic h;
        Plan p = solver.solve(l, s, h);
        assert(p.steps.size() <= 6);
    }


    {
        Level l;
        l.rows = 5; l.cols = 7;
        l.walls.assign(35, false);
        l.goals.assign(35, '\0');
        l.agent_colors.fill(Color::Unknown);
        l.box_colors.fill(Color::Unknown);
        l.agent_colors[0] = Color::Blue;
        l.agent_colors[1] = Color::Red;
        l.box_colors['A' - 'A'] = Color::Blue;
        l.box_colors['B' - 'A'] = Color::Red;
        l.goals[l.index(2,6)] = 'A';

        State s;
        s.rows = 5; s.cols = 7;
        s.agent_positions = {Position{2,0}, Position{0,0}};
        s.box_pos.assign(35, '\0');
        s.set_box(2,2,'A');
        s.set_box(1,4,'B');

        LevelAnalyzer analyzer;
        LevelAnalysis analysis = analyzer.analyze(l, s);
        for (Position candidate : analysis.parking_cells) {
            if (candidate == Position{2,4}) {
                analysis.at(candidate).parking_score = 10000;
            } else if (candidate == Position{0,4}) {
                analysis.at(candidate).parking_score = 100;
            } else {
                analysis.at(candidate).parking_score = 1;
            }
        }
        std::sort(analysis.parking_cells.begin(), analysis.parking_cells.end(), [&](Position a, Position b) {
            return analysis.at(a).parking_score > analysis.at(b).parking_score;
        });

        int next_task_id = 1;
        BlockerResolver resolver;
        auto tasks = resolver.generate_blocker_tasks(l, s, analysis, next_task_id);
        bool checked_b = false;
        for (const Task& task : tasks) {
            if (task.type == TaskType::MoveBlockingBoxToParking && task.box_id == 'B') {
                checked_b = true;
                assert(!(task.parking_pos == Position{2,4}));
                assert((task.parking_pos == Position{0,4}));
            }
        }
        assert(checked_b);
    }

    {
        Level l = make_level();
        State s = make_state();
        s.set_box(3,3,'B');
        TaskGenerator g;
        auto tasks = g.generate_delivery_tasks(l,s);
        assert(!tasks.empty());
        bool found_a = false;
        for (const auto& task : tasks) {
            if (task.type == TaskType::DeliverBoxToGoal && task.box_id == 'A' && task.agent_id == 0) {
                found_a = true;
                break;
            }
        }
        assert(found_a);
    }

    {
        Task t;
        t.task_id = 7; t.type = TaskType::DeliverBoxToGoal; t.box_id = 'A'; t.goal_symbol='A'; t.agent_id = 0; t.priority = 5;
        assert(t.describe() == "Task#7 DeliverBoxToGoal box=A goal=A agent=0 prio=5");
    }

    {
        Level l;
        l.rows = 3; l.cols = 5;
        l.walls.assign(15, false);
        l.goals.assign(15, '\0');
        l.agent_colors.fill(Color::Unknown);
        l.box_colors.fill(Color::Unknown);
        l.agent_colors[0] = Color::Blue;
        l.box_colors['A' - 'A'] = Color::Blue;
        State s;
        s.rows = 3; s.cols = 5;
        s.agent_positions = {Position{1,1}};
        s.box_pos.assign(15, '\0');
        s.set_box(1,2,'A');

        Task t; t.task_id=0; t.agent_id=0; t.box_id='A'; t.box_pos={1,2}; t.goal_pos={1,4};
        BoxTransportPlanner p;
        TaskPlan plan = p.plan(l,s,t);
        assert(plan.success);
        assert(!plan.primitive_actions.empty());
        assert((plan.box_trajectory.back() == Position{1,4}));
    }

    {
        Level l = make_level();
        State s = make_state();
        Task t; t.task_id=1; t.agent_id=2; t.box_id='A'; t.box_pos={1,2}; t.goal_pos={1,3};
        BoxTransportPlanner p;
        TaskPlan plan = p.plan(l,s,t);
        assert(!plan.success);
        assert(plan.failure_reason == "invalid_agent");
    }

    {
        Level l;
        l.rows = 3; l.cols = 4;
        l.walls.assign(12, false);
        l.walls[l.index(1,3)] = true;
        l.goals.assign(12, '\0');
        l.agent_colors.fill(Color::Unknown);
        l.box_colors.fill(Color::Unknown);
        State s;
        s.rows = 3; s.cols = 4;
        s.agent_positions = {Position{1,1}};
        s.box_pos.assign(12, '\0');
        s.set_box(1,2,'A');

        Task t; t.task_id=2; t.agent_id=0; t.box_id='A'; t.box_pos={1,2}; t.goal_pos={1,3};
        BoxTransportPlanner p;
        TaskPlan plan = p.plan(l,s,t);
        assert(!plan.success);
        assert(plan.failure_reason == "no_path_for_single_box");
    }

    std::cout << "phase2_tests passed\n";
    return 0;
}
