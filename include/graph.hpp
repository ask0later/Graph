#pragma once

#include <initializer_list> 
#include <vector>
#include <utility>      // for pair
#include <iomanip>      // for print
#include <iostream>
#include <algorithm>    // for max
#include <queue>        // for BFS

namespace graph {

    namespace details {
        enum class Color {
            Gray, White, Black
        }; // enum class Color
    }; // namespace details

    template <typename T>
    class Vertex final {
    public:
        Vertex(T data) : data_(data) {}

        void Set(T data) {
            data_ = data;
        }

        T Get() const {
            return data_;
        }
    private:
        T data_;
    }; // class Vertex

    template <typename T>
    class Edge final {
    public:
        Edge(T data) : data_(data) {}

        void Set(T data) {
            data_ = data;
        }

        T Get() const {
            return data_;
        }
    private:
        T data_;
    }; // class Edge

    template <typename VertexT, typename EdgeT>
    class Graph final {
    public:
        Graph(const std::initializer_list<std::pair<size_t, size_t>> &vertices_pairs) {
            for (auto &&pair : vertices_pairs)
                vertices_count_ = std::max(vertices_count_, std::max(pair.first, pair.second));
            
            bool odd = false;
            if (vertices_count_ % 2 != 0) {
                vertices_count_++;
                odd = true;
            }
                
            edges_count_ = 2 * vertices_pairs.size();
            length_ = vertices_count_ + edges_count_;

            vertices_.resize(vertices_count_, Vertex(VertexT()));
            edges_.resize(edges_count_, Edge{EdgeT()});

            indices_.resize(length_);
            next_edges_.resize(length_);
            prev_edges_.resize(length_);

            size_t edges_count = 0;

            for (auto &&pair : vertices_pairs) {
                indices_[pair.first  - 1U] = 0U;
                indices_[pair.second - 1U] = 0U;

                indices_[vertices_count_ + edges_count] = pair.first;
                indices_[vertices_count_ + edges_count + 1U] = pair.second;
                
                edges_count += 2;
            }

            if (odd) {
                indices_[vertices_count_ - 1U] = 0U;
                next_edges_[vertices_count_ - 1U] = vertices_count_ - 1U;
                prev_edges_[vertices_count_ - 1U] = vertices_count_ - 1U;
            }

            for (size_t i = vertices_count_, j = length_ - 1; i < length_; ++i, --j) {
                size_t tmp = indices_[i] - 1U;

                while (next_edges_[tmp] != indices_[i] - 1U && next_edges_[tmp] != 0U)
                    tmp = next_edges_[tmp];
                
                next_edges_[tmp] = i;
                next_edges_[i] = indices_[i] - 1U;

                tmp = indices_[j] - 1U;

                while (prev_edges_[tmp] != indices_[j] - 1U && prev_edges_[tmp] != 0U)
                    tmp = prev_edges_[tmp];
                
                prev_edges_[tmp] = j;
                prev_edges_[j] = indices_[j] - 1U;
            }
        }

        template <typename Visitor>
        void DepthFirstSearch(Visitor &visitor) const {
            std::vector<details::Color> colors{vertices_count_, details::Color::White};
            
            for (size_t i = 0; i < vertices_count_; ++i) {
                if (colors[i] == details::Color::White)
                    DepthFirstSearch(colors, i, visitor);
            }
        }

        template <typename Visitor>
        void BreadthFirstSearch(Visitor &visitor) const {
            std::vector<details::Color> colors{vertices_count_, details::Color::White};
            
            std::queue<size_t> queue;
            queue.push(0);
            colors[0] = details::Color::Black;

            while (!queue.empty()) {
                size_t current = queue.front();
                queue.pop();
                colors[current] = details::Color::Black;

                visitor.VisitVertex(current, *this);

                std::vector<size_t> neighbours;
                GetNeighboringVertices(neighbours, current);

                for (auto &&neighbour : neighbours) {
                    if (colors[neighbour - 1U] == details::Color::White)
                        queue.push(neighbour - 1U);
                }
            }
        }

        void GetNeighboringVertices(std::vector<size_t> &neighbours, size_t vertexIndex) const {
            if (vertexIndex > vertices_count_) {
                throw std::out_of_range("Indices vector out of range");
            }

            if (indices_[vertexIndex] != 0U) {
                throw std::out_of_range("The vertex does not exist");
            }

            size_t next_edge = next_edges_[vertexIndex];
            while (next_edge != vertexIndex) {
                neighbours.push_back(indices_[next_edge ^ 1U]);
                next_edge = next_edges_[next_edge];
            }
        }

        void Print() const;
        const std::vector<size_t> &GetIndices() const;
        const std::vector<size_t> &GetNextEdges() const;
        const std::vector<size_t> &GetPrevEdges() const;
    
    private:
        template <typename Visitor>
        void DepthFirstSearch(std::vector<details::Color> &colors, size_t vertex_index, Visitor &visitor) const {
            colors[vertex_index] = details::Color::Gray;
            
            std::vector<size_t> neighbours;
            GetNeighboringVertices(neighbours, vertex_index);

            visitor.VisitVertex(vertex_index, *this);
            
            for (auto &&neighbour : neighbours) {
                if (colors[neighbour - 1U] == details::Color::White)
                    DepthFirstSearch(colors, neighbour - 1U, visitor);
            }

            colors[vertex_index] = details::Color::Black;
        }

        size_t length_ = 0;
        size_t vertices_count_ = 0;
        size_t edges_count_ = 0;
        std::vector<Vertex<VertexT>> vertices_;
        std::vector<Edge<EdgeT>> edges_;
        std::vector<size_t> indices_;
        std::vector<size_t> next_edges_;
        std::vector<size_t> prev_edges_;
    }; // class Graph
 
    template <typename VertexT, typename EdgeT>
    void Graph<VertexT, EdgeT>::Print() const {
        const int width = 5;

        for (size_t i = 0; i < length_; ++i) {
            std::cout << std::right << std::setw(width) << i;
        }

        std::cout << std::endl;

        for (size_t i = 0; i < length_; ++i) {
            std::cout << std::right << std::setw(width) << indices_[i];
        }
        
        std::cout << std::endl;

        for (size_t i = 0; i < length_; ++i) {
            std::cout << std::right << std::setw(width) << next_edges_[i];
        }
        
        std::cout << std::endl;

        for (size_t i = 0; i < length_; ++i) {
            std::cout << std::right << std::setw(width) << prev_edges_[i];
        }

        std::cout << std::endl;
    }

    template <typename VertexT, typename EdgeT>
    const std::vector<size_t> &Graph<VertexT, EdgeT>::GetIndices() const {
        return indices_;
    }

    template <typename VertexT, typename EdgeT>
    const std::vector<size_t> &Graph<VertexT, EdgeT>::GetNextEdges() const {
        return next_edges_;
    }

    template <typename VertexT, typename EdgeT>
    const std::vector<size_t> &Graph<VertexT, EdgeT>::GetPrevEdges() const {
        return prev_edges_;
    }

}; // namespace graph