#pragma once

#include <initializer_list> 
#include <vector>
#include <utility>      // for pair
#include <iomanip>      // for print
#include <iostream>
#include <algorithm>    // for max
#include <queue>        // for BFS

namespace graph {
    enum class Color {
        Gray, White, Black
    }; // enum class Color

    class Graph final {
    public:
        Graph(const std::initializer_list<std::pair<size_t, size_t>> &vertices_pairs) {
            for (auto &&pair : vertices_pairs)
                vertices_count_ = std::max(vertices_count_, std::max(pair.first, pair.second));

            edges_count_ = vertices_pairs.size();
            length_ = vertices_count_ + 2 * edges_count_;

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

        void DepthFirstSearch() const {
            std::vector<Color> colors{vertices_count_, Color::White};

            for (size_t i = 0; i < vertices_count_; ++i) {
                if (colors[i] == Color::White)
                    DepthFirstSearch(colors, i);
            }
        }

        void BreadthFirstSearch() const {
            std::vector<Color> colors{vertices_count_, Color::White};
            
            std::queue<size_t> queue;
            queue.push(0);
            colors[0] = Color::Black;

            while (!queue.empty()) {
                size_t current = queue.front();
                queue.pop();
                colors[current] = Color::Black;
                // do an action

                std::vector<size_t> neighbours;
                GetNeighboringVertices(neighbours, current);

                for (auto &&neighbour : neighbours)
                    if (colors[neighbour] == Color::White)
                        queue.push(neighbour);
            }
        }

        void Print() const;
        const std::vector<size_t> &GetIndices() const;
        const std::vector<size_t> &GetNextEdges() const;
        const std::vector<size_t> &GetPrevEdges() const;
    
    private:
        void GetNeighboringVertices(std::vector<size_t> &neighbours, size_t vertex_index) const {
            if (vertex_index > vertices_count_) {
                throw std::out_of_range("Indices vector out of range");
            }

            if (indices_[vertex_index] != 0U) {
                throw std::out_of_range("The vertex does not exist");
            }

            size_t next_edge = next_edges_[vertex_index];
            while (next_edge != vertex_index) {
                neighbours.push_back(indices_[next_edge ^ 1]);
                next_edge = next_edges_[next_edge];
            }
        }

        void DepthFirstSearch(std::vector<Color> &colors, size_t vertex_index) const {
            colors[vertex_index] = Color::Gray;
            
            std::vector<size_t> neighbours;
            GetNeighboringVertices(neighbours, vertex_index);
            
            for (auto &&neighbour : neighbours)
                if (colors[neighbour] == Color:: White)
                    DepthFirstSearch(colors, neighbour);

            colors[vertex_index] = Color::Black;
            // do an action
        }

        size_t length_ = 0;
        size_t vertices_count_ = 0;
        size_t edges_count_ = 0;
        std::vector<size_t> indices_;
        std::vector<size_t> next_edges_;
        std::vector<size_t> prev_edges_;
    }; // class Graph
 
    void Graph::Print() const {
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

    const std::vector<size_t> &Graph::GetIndices() const {
        return indices_;
    }

    const std::vector<size_t> &Graph::GetNextEdges() const {
        return next_edges_;
    }

    const std::vector<size_t> &Graph::GetPrevEdges() const {
        return prev_edges_;
    }

}; // namespace graph