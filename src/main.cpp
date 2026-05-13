// src/main.cpp
#include "client/SearchClient.hpp"
#include "utils/ArgParser.hpp"

#include <iostream>
#include <exception>

static void printUsage() {
    std::cerr
        << "Usage: searchclient [options]\n"
        << "  --solver <bfs|dfs|astar|greedy|spacetime_astar|comp|lns_htn>\n"
        << "  --heuristic <none|goal_count|agent_goal_distance|box_goal_distance>\n"
        << "  --verbose\n"
        << "  --help\n";
}

int main(int argc, char** argv) {
    try {
        SearchConfig config = parseArgs(argc, argv);

        if (config.help) {
            printUsage();
            return 0;
        }

        SearchClient client(config);
        client.run();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Argument/config error: " << e.what() << '\n';
        printUsage();
        return 1;
    }
}