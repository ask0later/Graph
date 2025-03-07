#include "graph.hpp"
#include "bipartite.hpp"

#include <gtest/gtest.h>

TEST(GraphTest, GraphCtor1) {
    graph::Graph<int, int> graph {{1, 2}, {1, 3}, {2, 3}, {2, 4}, {3, 4}};

    std::vector<size_t> v1{0, 0, 0, 0, 1, 2, 1, 3, 2, 3, 2, 4, 3, 4};
    std::vector<size_t> v2{4, 5, 7, 11, 6, 8, 0, 9, 10, 12, 1, 13, 2, 3};
    std::vector<size_t> v3{6, 10, 12, 13, 0, 1, 4, 2, 5, 7, 8, 3, 9, 11};
    
    ASSERT_EQ(graph.GetIndices(), v1);
    ASSERT_EQ(graph.GetNextEdges(), v2);
    ASSERT_EQ(graph.GetPrevEdges(), v3);
}

TEST(GraphTest, GraphCtor2) {
    graph::Graph<int, int> graph {{1, 2}, {1, 3}};

    std::vector<size_t> v1{0, 0, 0, 0, 1, 2, 1, 3};
    std::vector<size_t> v2{4, 5, 7, 3, 6, 1, 0, 2};
    std::vector<size_t> v3{6, 5, 7, 3, 0, 1, 4, 2};
    
    ASSERT_EQ(graph.GetIndices(), v1);
    ASSERT_EQ(graph.GetNextEdges(), v2);
    ASSERT_EQ(graph.GetPrevEdges(), v3);
}

TEST(GraphTest, GraphBipartiteBFS1) {
    graph::Graph<int, int> graph {{1, 2}, {1, 3}};
    graph::BipartiteChecker<int, int> checker;
    bool isBipartite = checker.isBipartite(graph, graph::CheckingPolicy::BFS);
    ASSERT_EQ(isBipartite, true);
}

TEST(GraphTest, GraphBipartiteBFS2) {
    graph::Graph<int, int> graph {{1, 2}, {1, 3}, {4, 5}, {4, 2}, {1, 5}, {6, 7}, {6, 2}, {6, 3}, {7, 1}, {7, 4}};
    graph::BipartiteChecker<int, int> checker;
    bool isBipartite = checker.isBipartite(graph, graph::CheckingPolicy::BFS);
    ASSERT_EQ(isBipartite, true);
}

TEST(GraphTest, GraphBipartiteBFS3) {
    graph::Graph<int, int> graph {{1, 2}, {1, 3}, {4, 5}, {4, 2}, {1, 5}, {5, 2}};
    graph::BipartiteChecker<int, int> checker;
    bool isBipartite = checker.isBipartite(graph, graph::CheckingPolicy::BFS);
    ASSERT_EQ(isBipartite, false);
}

TEST(GraphTest, GraphBipartiteDFS1) {
    graph::Graph<int, int> graph {{1, 2}, {1, 3}};
    graph::BipartiteChecker<int, int> checker;
    bool isBipartite = checker.isBipartite(graph, graph::CheckingPolicy::DFS);
    ASSERT_EQ(isBipartite, true);
}

TEST(GraphTest, GraphBipartiteDFS2) {
    graph::Graph<int, int> graph {{1, 2}, {1, 3}, {4, 5}, {4, 2}, {1, 5}, {6, 7}, {6, 2}, {6, 3}, {7, 1}, {7, 4}};
    graph::BipartiteChecker<int, int> checker;
    bool isBipartite = checker.isBipartite(graph, graph::CheckingPolicy::DFS);
    ASSERT_EQ(isBipartite, true);
}

TEST(GraphTest, GraphBipartiteDFS3) {
    graph::Graph<int, int> graph {{1, 2}, {1, 3}, {4, 5}, {4, 2}, {1, 5}, {5, 2}};
    graph::BipartiteChecker<int, int> checker;
    bool isBipartite = checker.isBipartite(graph, graph::CheckingPolicy::DFS);
    ASSERT_EQ(isBipartite, false);
}