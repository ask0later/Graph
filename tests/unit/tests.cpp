#include "graph.hpp"
#include "bipartite.hpp"
#include <list>

#include <gtest/gtest.h>

TEST(GraphTest, GraphCtor1) {
    graph::Graph<int, int> graph {{1, 2}, {1, 3}, {2, 3}, {2, 4}, {3, 4}};

    std::vector<size_t> v1{0, 0, 0, 0, 1, 2, 1, 3, 2, 3, 2, 4, 3, 4};
    std::vector<size_t> v2{4, 5, 7, 11, 6, 8, 0, 9, 10, 12, 1, 13, 2, 3};
    std::vector<size_t> v3{6, 10, 12, 13, 0, 1, 4, 2, 5, 7, 8, 3, 9, 11};
    
    size_t i = 0;
    for (auto &&elem : graph.GetIndices())
        ASSERT_EQ(elem, v1[i++]); i = 0;
    
    for (auto &&elem : graph.GetNextEdges())
        ASSERT_EQ(elem, v2[i++]); i = 0;
    
    for (auto &&elem : graph.GetPrevEdges())
        ASSERT_EQ(elem, v3[i++]);
}

TEST(GraphTest, GraphCtor2) {
    graph::Graph<int, int> graph {{1, 2}, {1, 3}};

    std::vector<size_t> v1{0, 0, 0, 0, 1, 2, 1, 3};
    std::vector<size_t> v2{4, 5, 7, 3, 6, 1, 0, 2};
    std::vector<size_t> v3{6, 5, 7, 3, 0, 1, 4, 2};
    
    size_t i = 0;
    for (auto &&elem : graph.GetIndices())
        ASSERT_EQ(elem, v1[i++]); i = 0;
    
    for (auto &&elem : graph.GetNextEdges())
        ASSERT_EQ(elem, v2[i++]); i = 0;
    
    for (auto &&elem : graph.GetPrevEdges())
        ASSERT_EQ(elem, v3[i++]);
}

TEST(GraphTest, GraphCtor3) {
    std::vector<std::pair<size_t, size_t>> v;
    v.push_back({1, 2});
    v.push_back({1, 3});
    v.push_back({2, 3});
    v.push_back({2, 4});
    v.push_back({3, 4});
    graph::Graph<int, int> graph1(v);
    graph::Graph<int, int> graph2(v.begin(), v.end());

    std::vector<size_t> v1{0, 0, 0, 0, 1, 2, 1, 3, 2, 3, 2, 4, 3, 4};
    std::vector<size_t> v2{4, 5, 7, 11, 6, 8, 0, 9, 10, 12, 1, 13, 2, 3};
    std::vector<size_t> v3{6, 10, 12, 13, 0, 1, 4, 2, 5, 7, 8, 3, 9, 11};
    
    size_t i = 0;
    for (auto &&elem : graph1.GetIndices())
        ASSERT_EQ(elem, v1[i++]); i = 0;
    
    for (auto &&elem : graph1.GetNextEdges())
        ASSERT_EQ(elem, v2[i++]); i = 0;
    
    for (auto &&elem : graph1.GetPrevEdges())
        ASSERT_EQ(elem, v3[i++]); i = 0;

    for (auto &&elem : graph2.GetIndices())
        ASSERT_EQ(elem, v1[i++]); i = 0;
    
    for (auto &&elem : graph2.GetNextEdges())
        ASSERT_EQ(elem, v2[i++]); i = 0;
    
    for (auto &&elem : graph2.GetPrevEdges())
        ASSERT_EQ(elem, v3[i++]); i = 0;
}

TEST(GraphTest, GraphCtor4) {
    std::list<std::pair<size_t, size_t>> l;
    l.push_back({1, 2});
    l.push_back({1, 3});
    l.push_back({2, 3});
    l.push_back({2, 4});
    l.push_back({3, 4});
    graph::Graph<int, int> graph1(l);
    graph::Graph<int, int> graph2(l.begin(), l.end());

    std::vector<size_t> v1{0, 0, 0, 0, 1, 2, 1, 3, 2, 3, 2, 4, 3, 4};
    std::vector<size_t> v2{4, 5, 7, 11, 6, 8, 0, 9, 10, 12, 1, 13, 2, 3};
    std::vector<size_t> v3{6, 10, 12, 13, 0, 1, 4, 2, 5, 7, 8, 3, 9, 11};
    
    size_t i = 0;
    for (auto &&elem : graph1.GetIndices())
        ASSERT_EQ(elem, v1[i++]); i = 0;
    
    for (auto &&elem : graph1.GetNextEdges())
        ASSERT_EQ(elem, v2[i++]); i = 0;
    
    for (auto &&elem : graph1.GetPrevEdges())
        ASSERT_EQ(elem, v3[i++]); i = 0;

    for (auto &&elem : graph2.GetIndices())
        ASSERT_EQ(elem, v1[i++]); i = 0;
    
    for (auto &&elem : graph2.GetNextEdges())
        ASSERT_EQ(elem, v2[i++]); i = 0;
    
    for (auto &&elem : graph2.GetPrevEdges())
        ASSERT_EQ(elem, v3[i++]); i = 0;
}

TEST(GraphTest, GraphBipartiteBFS1) {
    graph::Graph<int, int> graph {{1, 2}, {1, 3}};
    graph::BipartiteVisitor<int, int> visitor(graph.GetVerticesCount());
    graph::BipartiteChecker<int, int> checker(visitor);
    bool isBipartite = checker.isBipartite(graph, graph::CheckingPolicy::BFS);
    ASSERT_EQ(isBipartite, true);
}

TEST(GraphTest, GraphBipartiteBFS2) {
    graph::Graph<int, int> graph {{1, 2}, {1, 3}, {4, 5}, {4, 2}, {1, 5}, {6, 7}, {6, 2}, {6, 3}, {7, 1}, {7, 4}};
    graph::BipartiteVisitor<int, int> visitor(graph.GetVerticesCount());
    graph::BipartiteChecker<int, int> checker(visitor);
    bool isBipartite = checker.isBipartite(graph, graph::CheckingPolicy::BFS);
    ASSERT_EQ(isBipartite, true);
}

TEST(GraphTest, GraphBipartiteBFS3) {
    graph::Graph<int, int> graph {{1, 2}, {1, 3}, {4, 5}, {4, 2}, {1, 5}, {5, 2}};
    graph::BipartiteVisitor<int, int> visitor(graph.GetVerticesCount());
    graph::BipartiteChecker<int, int> checker(visitor);
    bool isBipartite = checker.isBipartite(graph, graph::CheckingPolicy::BFS);
    ASSERT_EQ(isBipartite, false);
}

TEST(GraphTest, GraphBipartiteDFS1) {
    graph::Graph<int, int> graph {{1, 2}, {1, 3}};
    graph::BipartiteVisitor<int, int> visitor(graph.GetVerticesCount());
    graph::BipartiteChecker<int, int> checker(visitor);
    bool isBipartite = checker.isBipartite(graph, graph::CheckingPolicy::DFS);
    ASSERT_EQ(isBipartite, true);
}

TEST(GraphTest, GraphBipartiteDFS2) {
    graph::Graph<int, int> graph {{1, 2}, {1, 3}, {4, 5}, {4, 2}, {1, 5}, {6, 7}, {6, 2}, {6, 3}, {7, 1}, {7, 4}};
    graph::BipartiteVisitor<int, int> visitor(graph.GetVerticesCount());
    graph::BipartiteChecker<int, int> checker(visitor);
    bool isBipartite = checker.isBipartite(graph, graph::CheckingPolicy::DFS);
    ASSERT_EQ(isBipartite, true);
}

TEST(GraphTest, GraphBipartiteDFS3) {
    graph::Graph<int, int> graph {{1, 2}, {1, 3}, {4, 5}, {4, 2}, {1, 5}, {5, 2}};
    graph::BipartiteVisitor<int, int> visitor(graph.GetVerticesCount());
    graph::BipartiteChecker<int, int> checker(visitor);
    bool isBipartite = checker.isBipartite(graph, graph::CheckingPolicy::DFS);
    ASSERT_EQ(isBipartite, false);
}