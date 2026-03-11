#include "client/SearchClient.hpp"
#include "parser/LevelParser.hpp"
#include "solvers/Solver.hpp"
#include <iostream>
#include <memory>

// Temporary factory declaration from NaiveSolver.cpp
std::unique_ptr<Solver> make_naive_solver();

void SearchClient::run() {
    auto level = LevelParser::parse(std::cin);
    auto solver = make_naive_solver();
    auto plan = solver->solve(level);

    for (const auto& joint_action : plan.steps) {
        std::cout << joint_action.to_string() << std::endl;
    }
}