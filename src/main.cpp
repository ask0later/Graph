#include "graph.hpp"
#include "bipartite.hpp"
#include <iostream>

int main() try {
    graph::Graph<int, int> graph;
    std::cin >> graph;
    
    graph::BipartiteVisitor<int, int> visitor;
    graph::BipartiteChecker<int, int> checker(visitor);
    
    if (!checker.isBipartite(graph, graph::CheckingPolicy::DFS)) {
        std::cout << "Graph is not bipartite" << std::endl;
        return 0;
    }
    
    std::string output;
    size_t prev = 0U;
    for (auto&& elem : checker.GetColors()) {
        while (prev < elem.first)
            output += std::to_string(prev++ + 1U) + " b ";

        output += std::to_string(elem.first + 1) + " ";
        output += (elem.second == 0) ? "b " : "r ";
        prev++;
    }
    output.erase(std::prev(output.end()));
    std::cout << output << std::endl;
} catch (std::exception &ex) {
    std::cout << ex.what() << std::endl;
};
