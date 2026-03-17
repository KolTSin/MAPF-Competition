// include/search/solvers/SolverFactory.hpp
#pragma once

#include "client/SearchConfig.hpp"
#include "solvers/Solver.hpp"
#include "search/heuristics/Heuristic.hpp"
#include <memory>

std::unique_ptr<Solver> makeSolver(
    SolverType solverType
    // const IHeuristic* heuristic
);