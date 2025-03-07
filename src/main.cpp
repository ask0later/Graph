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
    for (auto&& elem : checker.GetColors()) {
        output += std::to_string(elem.first + 1) + " ";
        output += (elem.second == 0) ? "b " : "r ";
    }
    output.erase(std::prev(output.end()));
    std::cout << output << std::endl;
} catch (std::exception &ex) {
    std::cout << ex.what() << std::endl;
};
