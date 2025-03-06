#pragma once

#include <initializer_list> 
#include <vector>
#include <utility> // for pair
#include <iomanip> // for print
#include <iostream>
#include <algorithm> // for max

namespace graph {

    class Graph final {
    public:
        Graph(const std::initializer_list<std::pair<size_t, size_t>> &vertices_pairs) {
            size_t vertices_count = 0;
            size_t pair_count = 0;
            for (auto &&pair : vertices_pairs) {
                vertices_count = std::max(vertices_count, std::max(pair.first, pair.second));
                pair_count++;
            }

            length_ = vertices_count + 2 * pair_count;

            indices_.resize(length_);
            next_ribs_.resize(length_);
            prev_ribs_.resize(length_);

            size_t ribs_count = 0;

            for (auto &&pair : vertices_pairs) {
                indices_[pair.first  - 1U] = 0U;
                indices_[pair.second - 1U] = 0U;

                indices_[vertices_count + ribs_count] = pair.first;
                indices_[vertices_count + ribs_count + 1U] = pair.second;
                
                ribs_count += 2;
            }

            for (size_t i = vertices_count, j = length_ - 1; i < length_; ++i, --j) {
                size_t tmp = indices_[i] - 1U;

                while (next_ribs_[tmp] != indices_[i] - 1U && next_ribs_[tmp] != 0U) {
                    tmp = next_ribs_[tmp];
                }
                
                next_ribs_[tmp] = i;
                next_ribs_[i] = indices_[i] - 1U;

                tmp = indices_[j] - 1U;

                while (prev_ribs_[tmp] != indices_[j] - 1U && prev_ribs_[tmp] != 0U) {
                    tmp = prev_ribs_[tmp];
                }
                
                prev_ribs_[tmp] = j;
                prev_ribs_[j] = indices_[j] - 1U;
            }
        }

        void Print() const;
        const std::vector<size_t> &GetIndeces() const;
        const std::vector<size_t> &GetNextRibs() const;
        const std::vector<size_t> &GetPrevRibs() const;
    private:
        size_t length_;
        std::vector<size_t> indices_;
        std::vector<size_t> next_ribs_;
        std::vector<size_t> prev_ribs_;
    }; // class Graph
 

    void Graph::Print() const {
        {
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
                std::cout << std::right << std::setw(width) << next_ribs_[i];
            }
            
            std::cout << std::endl;

            for (size_t i = 0; i < length_; ++i) {
                std::cout << std::right << std::setw(width) << prev_ribs_[i];
            }

            std::cout << std::endl;
        }
    }

    const std::vector<size_t> &Graph::GetIndeces() const {
        return indices_;
    }

    const std::vector<size_t> &Graph::GetNextRibs() const {
        return next_ribs_;
    }
    const std::vector<size_t> &Graph::GetPrevRibs() const {
        return prev_ribs_;
    }
    
}; // namespace graph