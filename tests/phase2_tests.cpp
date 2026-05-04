#include "tasks/TaskGenerator.hpp"
#include "tasks/Task.hpp"
#include "tasks/TaskScheduler.hpp"
#include "tasks/DependencyBuilder.hpp"
#include "hospital/BoxTransportPlanner.hpp"
#include "solvers/CompetitiveSolver.hpp"
#include "search/heuristics/Heuristic.hpp"

#include <cassert>
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

class ConstantHeuristic : public IHeuristic {
public:
    int evaluate(const State&) const override { return 0; }
    std::string name() const override { return "constant"; }
};

int main() {
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
        assert(graph.predecessors[2].size() == 1);
        assert(graph.predecessors[3].size() == 1);

        std::unordered_map<int, int> remaining;
        for (const auto& [task_id, preds] : graph.predecessors) remaining[task_id] = static_cast<int>(preds.size());
        auto newly_ready = graph.mark_completed_and_get_new_ready(1, remaining);
        assert(newly_ready.size() == 1 && newly_ready[0] == 2);
        auto no_dupes = builder.build({a,b,c,d});
        assert(no_dupes[2].size() == 1);
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
        assert(!p.empty());
        assert(p.steps.size() == 1);
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
        assert(!p.empty());
        assert(p.steps.size() <= 6);
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
