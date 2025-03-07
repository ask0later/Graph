#include "graph.hpp"
#include "bipartite.hpp"
#include <iostream>

int main() try {
    graph::Graph<int, int> graph;
    std::cin >> graph;
    
    graph::BipartiteVisitor<int, int> visitor;
    graph::BipartiteChecker<int, int> checker(visitor);
    
    if (!checker.isBipartite(graph, graph::CheckingPolicy::DFS))
        throw std::runtime_error("Graph is not bipartite");
    
    std::string output;
    for (auto&& elem : visitor) {
        output += std::to_string(elem.first + 1);
        output += (elem.second == 0) ? " b " : " r ";
    }
    std::cout << output << std::endl;
} catch (std::exception &ex) {
    std::cout << ex.what() << std::endl;
};
