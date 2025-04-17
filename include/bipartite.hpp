#pragma once

#include "graph.hpp"
#include <map>
#include <unordered_set>
#include <set>

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
        BipartiteVisitor(size_t verticesCount) : isBipartite_(true), verticesCount_(verticesCount), previousVertices_(verticesCount, -1) {
            colors_[0] = 0;
        }

        void VisitVertex(size_t vertexIndex, const Graph<VertexT, EdgeT> &graph) override {
            if (!isBipartite_)
                return;
            
            for (auto &&neighborIndex : graph.GetNeighboringVertices(vertexIndex)) {
                if (colors_.count(neighborIndex - 1U) == 0) {
                    previousVertices_[neighborIndex - 1U] = vertexIndex;
                    colors_[neighborIndex - 1U] = 1 - colors_[vertexIndex];
                } else if (colors_[neighborIndex - 1U] == colors_[vertexIndex]) {
                    isBipartite_ = false;
                    FindOddCycle(vertexIndex, neighborIndex - 1U);

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

        std::vector<size_t> &GetOddCycle() {
            return cycle_;
        }

    private:
        void FindOddCycle(size_t first, size_t second) {
            std::unordered_set<int> visited;
            
            for (int i = first; i != -1; i = previousVertices_[i])
                visited.insert(i);

            std::vector<int> tempPath;
            int commonVertex = -1;
            for (int i = second; i != -1; i = previousVertices_[i]) {
                tempPath.push_back(i + 1U);
                if (visited.count(i)) {
                    commonVertex = i;
                    break;
                }
            }
            cycle_.insert(cycle_.end(), tempPath.rbegin(), tempPath.rend());

            for (; first != commonVertex; first = previousVertices_[first])
                cycle_.push_back(first + 1U);
        }
        size_t verticesCount_;
        bool isBipartite_ = true;
        std::map<size_t, char> colors_;
        std::vector<size_t> cycle_;
        std::vector<int> previousVertices_;
    }; // class BipartiteVisitor

    enum class CheckingPolicy {
        BFS = 0,
        DFS = 1
    }; // enum CheckingPolicy

    template <typename VertexT, typename EdgeT>
    class BipartiteChecker {
    public:
        BipartiteChecker(BipartiteVisitor<VertexT, EdgeT> &visitor) : visitor_(visitor) {}

        bool isBipartite(const Graph<VertexT, EdgeT> &graph, CheckingPolicy policy) {
            size_t verticesCount = graph.GetVerticesCount();
            std::set<size_t> marked{verticesCount};

            auto &&searchFunction = (policy == CheckingPolicy::BFS) ?\
                DoBreadthFirstSearch<Graph<VertexT, EdgeT>, BipartiteVisitor<VertexT, EdgeT>> :\
                DoDepthFirstSearch<Graph<VertexT, EdgeT>, BipartiteVisitor<VertexT, EdgeT>>;

            if (policy != CheckingPolicy::BFS && policy != CheckingPolicy::DFS) {
                throw std::logic_error("Incorrect checking policy");
            }

            for (size_t i = 0; i < verticesCount; ++i) {
                auto &&vertex = static_cast<typename Graph<VertexT, EdgeT>::VertexIndex>(i);
                if (marked.count(vertex) == 0) {
                    auto&& temp = searchFunction(graph, vertex, visitor_);
                    marked.insert(temp.begin(), temp.end());
                }
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