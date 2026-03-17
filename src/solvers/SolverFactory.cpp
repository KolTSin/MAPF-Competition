// src/solvers/SolverFactory.cpp
#include "client/SearchConfig.hpp"
#include "solvers/Solver.hpp"
#include "solvers/NaiveSolver.hpp"
#include "solvers/SequentialSolver.hpp"
#include "solvers/SpaceTimeSolver.hpp"
// #include "search/heuristics/Heuristic.hpp"

#include <memory>
#include <stdexcept>

std::unique_ptr<Solver> makeSolver(
    SolverType solverType
    // const IHeuristic* heuristic
) {
    switch (solverType) {
        case SolverType::NAIVE:
            // if (!heuristic) throw std::runtime_error("NaiveSolver requires a heuristic");
            return std::make_unique<NaiveSolver>();

        case SolverType::SEQUENTIAL:
            // if (!heuristic) throw std::runtime_error("SequentialSolver requires a heuristic");
            return std::make_unique<SequentialSolver>();

        case SolverType::SPACETIME_ASTAR:
            // if (!heuristic) throw std::runtime_error("SpaceTimeAStar requires a heuristic");
            return std::make_unique<SpaceTimeSolver>();

        default:
            throw std::runtime_error("Unsupported solver type");
    }
}