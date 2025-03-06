#include "graph.hpp"

int main() try {
    graph::Graph<int, int> graph1 {{1, 2}, {1, 3}, {2, 3}, {2, 4}, {3, 4}};
    graph1.Print();
    graph::Graph<int, int> graph2 {{1, 2}, {1, 3}, {2, 3}, {2, 4}, {3, 4}, {3, 5}};
    graph2.Print();
} catch (std::exception &ex) {
    std::cout << ex.what() << std::endl;
};