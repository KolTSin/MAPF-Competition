#include "client/SearchClient.hpp"
#include <exception>
#include <iostream>

int main() {
    try {
        SearchClient client;
        client.run();
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}