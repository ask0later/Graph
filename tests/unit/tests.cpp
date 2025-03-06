#include "graph.hpp"

#include <gtest/gtest.h>

TEST(GraphTest, GraphCtor) {
    graph::Graph graph {{1, 2}, {1, 3}, {2, 3}, {2, 4}, {3, 4}};

    std::vector<size_t> v1{0, 0, 0, 0, 1, 2, 1, 3, 2, 3, 2, 4, 3, 4};
    std::vector<size_t> v2{4, 5, 7, 11, 6, 8, 0, 9, 10, 12, 1, 13, 2, 3};
    std::vector<size_t> v3{6, 10, 12, 13, 0, 1, 4, 2, 5, 7, 8, 3, 9, 11};
    
    ASSERT_EQ(graph.GetIndeces(), v1);
    ASSERT_EQ(graph.GetNextRibs(), v2);
    ASSERT_EQ(graph.GetPrevRibs(), v3);
}