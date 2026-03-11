#include "client/SearchClient.hpp"
#include "parser/LevelParser.hpp"
#include "solvers/NaiveSolver.hpp"
#include <iostream>
#include <memory>


void SearchClient::run() {
    auto level = LevelParser::parse(std::cin);
    NaiveSolver solver;
    auto plan = solver.solve(level);

    for (const auto& joint_action : plan.steps) {
        std::cout << joint_action.to_string() << std::endl;
    }
}