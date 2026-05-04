#include "tasks/TaskGenerator.hpp"
#include "tasks/Task.hpp"
#include "hospital/BoxTransportPlanner.hpp"

#include <cassert>
#include <iostream>

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

int main() {
    {
        Level l = make_level();
        State s = make_state();
        s.set_box(3,3,'B');
        TaskGenerator g;
        auto tasks = g.generate_delivery_tasks(l,s);
        assert(tasks.size() == 1);
        assert(tasks[0].box_id == 'A');
        assert(tasks[0].agent_id == 0);
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

    std::cout << "phase2_tests passed\n";
    return 0;
}
