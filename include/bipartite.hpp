#pragma once

#include "graph.hpp"
#include <map>

namespace graph {

template <typename VertexT, typename EdgeT>
    class GraphVisitor {
    public:
        virtual void VisitVertex(size_t vertexIndex, const Graph<VertexT, EdgeT> &graph) = 0;
        virtual ~GraphVisitor() = default;
    }; // class GraphVisitor

    template <typename VertexT, typename EdgeT>
    class BipartiteChecker : public GraphVisitor<VertexT, EdgeT> {
    public:
        BipartiteChecker() {
            colors_[0] = 0;
            isBipartite_ = true;
        }

        void VisitVertex(size_t vertexIndex, const Graph<VertexT, EdgeT> &graph) override {
            if (!isBipartite_)
                return;
            
            std::vector<size_t> neighborsIndices;
            graph.GetNeighboringVertices(neighborsIndices, vertexIndex);

            for (auto&& neighborIndex : neighborsIndices) {
                if (colors_.count(neighborIndex - 1) == 0) {
                    colors_[neighborIndex - 1] = 1 - colors_[vertexIndex];
                }
                else if (colors_[neighborIndex - 1] == colors_[vertexIndex]) {
                    isBipartite_ = false;
                    return;
                }
            }
        }

        void PrintColors() const {
            if (colors_.size() < 1)
                throw std::runtime_error("Colors are empty");
            if (!isBipartite_)
                throw std::runtime_error("Graph is not bipartite");

            for (auto&& elem : colors_) {
                char color = elem.second;
                std::cout << elem.first + 1 << " ";
                if (color == 0)
                    std::cout << "b ";
                else
                    std::cout << "r ";
            }
            std::cout << std::endl;
        }

        bool isBipartite() const {
            return isBipartite_;
        }

    private:
        bool isBipartite_ = true;
        std::map<size_t, char> colors_;
    }; // class BipartiteChecker


} // namespace graph