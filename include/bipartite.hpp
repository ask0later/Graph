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
    class BipartiteVisitor : public GraphVisitor<VertexT, EdgeT> {
    public:
        BipartiteVisitor() {
            colors_[0] = 0;
            isBipartite_ = true;
        }

        void VisitVertex(size_t vertexIndex, const Graph<VertexT, EdgeT> &graph) override {
            if (!isBipartite_)
                return;
            
            std::vector<size_t> neighborsIndices;
            graph.GetNeighboringVertices(neighborsIndices, vertexIndex);

            for (auto&& neighborIndex : neighborsIndices) {
                if (colors_.count(neighborIndex - 1U) == 0) {
                    colors_[neighborIndex - 1U] = 1 - colors_[vertexIndex];
                } else if (colors_[neighborIndex - 1U] == colors_[vertexIndex]) {
                    isBipartite_ = false;
                    return;
                }
            }
        }

        inline auto GetColors() const {
            return colors_;
        }

        bool isBipartite() const {
            if (colors_.size() < 1)
                throw std::runtime_error("Colors are empty");

            return isBipartite_;
        }

    private:
        bool isBipartite_ = true;
        std::map<size_t, char> colors_;
    }; // class BipartiteVisitor

    enum class CheckingPolicy {
        BFS = 0,
        DFS = 1
    }; // enum CheckingPolicy

    template <typename VertexT, typename EdgeT>
    class BipartiteChecker {
    public:
        BipartiteChecker(BipartiteVisitor<VertexT, EdgeT> &visitor) : visitor_(visitor) {}

        bool isBipartite(const Graph<VertexT, EdgeT> &graph, CheckingPolicy policy) const {
            switch (policy) {
                case CheckingPolicy::BFS:
                    graph.BreadthFirstSearch(visitor_);
                    break;
                case CheckingPolicy::DFS:
                    graph.DepthFirstSearch(visitor_);
                    break;
                default:
                    throw std::logic_error("Incorrect checking policy");
            }
            
            return visitor_.isBipartite();
        }

        inline auto GetColors() const {
            return visitor_.GetColors();
        }

    private:
        BipartiteVisitor<VertexT, EdgeT> &visitor_;
    }; // BipartiteChecker


} // namespace graph