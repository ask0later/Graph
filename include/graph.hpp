#pragma once

#include <initializer_list>
#include <vector>
#include <utility>          // for pair
#include <iomanip>          // for print
#include <iostream>
#include <algorithm>        // for max
#include <queue>            // for BFS
#include <stack>            // for DFS
#include <tuple>
#include <cassert>
#include <span>
#include <concepts>
#include <iterator>
#include <set>

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
        Edge(T data, size_t fVertex = 0, size_t sVertex = 0) : data_(data), fVertex_(fVertex), sVertex_(sVertex) {}

        void Set(T data) {
            data_ = data;
        }

        T Get() const {
            return data_;
        }

        size_t GetFirstVertex() const {
            return fVertex_;
        }

        size_t GetSecondVertex() const {
            return sVertex_;
        }
    private:
        T data_;
        size_t fVertex_, sVertex_;
    }; // class Edge

    template <typename VertexT, typename EdgeT>
    class Graph final {
    public:
        using VertexIndex = size_t;
        using VertexPair = std::pair<VertexIndex, VertexIndex>;
        using VertexPairWithEdge = std::tuple<VertexIndex, VertexIndex, EdgeT>;
    public:

        Graph() = default;
        
        Graph(const std::initializer_list<VertexPair> &data) {
            for (auto &&pair : data) {
                edges_.emplace_back(EdgeT(), pair.first, pair.second);
                verticesCount_ = std::max(verticesCount_, std::max(pair.first, pair.second));                
            }

            edgesCount_ = edges_.size();
            FillTable();
        }

        Graph(const std::initializer_list<VertexPairWithEdge> &data) {
            for (auto &&tuple : data) {
                auto fVertexIndex = std::get<0>(tuple), sVertexIndex = std::get<1>(tuple);
                edges_.emplace_back(std::get<EdgeT>(tuple), fVertexIndex, sVertexIndex);
                verticesCount_ = std::max(verticesCount_, std::max(fVertexIndex, sVertexIndex));
            }
            
            edgesCount_ = edges_.size();
            FillTable();
        }

        template <std::input_iterator Iterator>
        requires std::is_same_v<std::iter_value_t<Iterator>, VertexPair>
        Graph(Iterator begin, Iterator end) {
            for (auto it = begin; it != end; ++it) {
                edges_.emplace_back(EdgeT(), it->first, it->second);
                verticesCount_ = std::max(verticesCount_, std::max(it->first, it->second));                
            }

            edgesCount_ = edges_.size();
            FillTable();
        }

        template <std::input_iterator Iterator>
        requires std::is_same_v<std::iter_value_t<Iterator>, VertexPairWithEdge>
        Graph(Iterator begin, Iterator end) {
            for (auto it = begin; it != end; ++it) {
                auto fVertexIndex = std::get<0>(*it), sVertexIndex = std::get<1>(*it);
                edges_.emplace_back(std::get<EdgeT>(*it), fVertexIndex, sVertexIndex);
                verticesCount_ = std::max(verticesCount_, std::max(fVertexIndex, sVertexIndex));
            }

            edgesCount_ = edges_.size();
            FillTable();
        }

        template <typename Container>
        requires std::is_same_v<std::iter_value_t<typename Container::iterator>, VertexPair>
        Graph(const Container &data) {
            for (auto &&pair : data) {
                edges_.emplace_back(EdgeT(), pair.first, pair.second);
                verticesCount_ = std::max(verticesCount_, std::max(pair.first, pair.second));                
            }

            edgesCount_ = edges_.size();
            FillTable();
        }

        template <typename Container>
        requires std::is_same_v<std::iter_value_t<typename Container::iterator>, VertexPairWithEdge>
        Graph(const Container &data) {
            for (auto &&tuple : data) {
                auto fVertexIndex = std::get<0>(tuple), sVertexIndex = std::get<1>(tuple);
                edges_.emplace_back(std::get<EdgeT>(tuple), fVertexIndex, sVertexIndex);
                verticesCount_ = std::max(verticesCount_, std::max(fVertexIndex, sVertexIndex));
            }

            edgesCount_ = edges_.size();
            FillTable();
        }

        std::vector<size_t> GetNeighboringVertices(size_t vertexIndex) const {
            if (vertexIndex > verticesCount_) {
                throw std::out_of_range("Indices vector out of range");
            }

            if (indices_[vertexIndex] != 0U) {
                throw std::out_of_range("The vertex does not exist");
            }

            std::vector<size_t> neighbours;
            size_t nextEdge = nextEdges_[vertexIndex];
            
            while (nextEdge != vertexIndex) {
                neighbours.push_back(indices_[nextEdge ^ 1U]);
                nextEdge = nextEdges_[nextEdge];
            }

            return neighbours;
        }

        friend std::ostream &operator<<(std::ostream &out, const Graph<VertexT, EdgeT> &graph) {
            return graph.Print(out);
        }

        friend std::istream &operator>>(std::istream &in, Graph<VertexT, EdgeT> &graph) {
            return graph.Read(in);
        }

        size_t GetVerticesCount() const {
            return verticesCount_;
        }
        
        std::span<const size_t> GetIndices() const;
        std::span<const size_t> GetNextEdges() const;
        std::span<const size_t> GetPrevEdges() const;
    
    private:
        std::ostream &Print(std::ostream &out) const;
        std::istream &Read(std::istream &in);

        void FillTable();
        void SetNextAndPrevEdges();
        void AddVertexToTable(size_t vertexIndex);
        void AddVertexToTable(size_t vertexIndex, const VertexT &val);
        void AddEdgeToTable(std::pair<size_t, size_t> &&verticesPair);

        size_t length_ = 0;
        size_t verticesCount_ = 0;
        size_t edgesCount_ = 0;
        size_t curEdgesCount_ = 0;
        std::vector<Vertex<VertexT>> vertices_;
        std::vector<Edge<EdgeT>> edges_;
        std::vector<size_t> indices_;
        std::vector<size_t> nextEdges_;
        std::vector<size_t> prevEdges_;
    }; // class Graph
 
    template <typename VertexT, typename EdgeT>
    std::ostream &Graph<VertexT, EdgeT>::Print(std::ostream &out) const {
        constexpr int width = 5;

        for (size_t i = 0; i < length_; ++i)
            out << std::right << std::setw(width) << i; out << std::endl;

        for (size_t i = 0; i < length_; ++i)
            out << std::right << std::setw(width) << indices_[i]; out << std::endl;

        for (size_t i = 0; i < length_; ++i)
            out << std::right << std::setw(width) << nextEdges_[i]; out << std::endl;

        for (size_t i = 0; i < length_; ++i)
            std::cout << std::right << std::setw(width) << prevEdges_[i];

        return out;
    }

    template <typename VertexT, typename EdgeT>
    std::istream &Graph<VertexT, EdgeT>::Read(std::istream &in) {
        int fVertexIndex = 0, sVertexIndex = 0;
        int edgeWeight = 0;
        std::string str;
        char dummy;
        while (!in.eof()) {
            if (!(in >> fVertexIndex >> str >> sVertexIndex >> dummy >> edgeWeight &&
                str == "--" && dummy == ','))
                throw std::runtime_error("Incorrect input");

            if (fVertexIndex <= 0 || sVertexIndex <= 0)
                throw std::runtime_error("Vertex index must be greater than zero");
            
            in >> std::ws;

            edges_.emplace_back(edgeWeight ,fVertexIndex, sVertexIndex);
            verticesCount_ = std::max(verticesCount_, static_cast<size_t>(std::max(fVertexIndex, sVertexIndex))); 
        }
        
        edgesCount_ = edges_.size();
        FillTable();
        return in;
    }

    template <typename VertexT, typename EdgeT>
    std::span<const size_t> Graph<VertexT, EdgeT>::GetIndices() const {
        return indices_;
    }

    template <typename VertexT, typename EdgeT>
    std::span<const size_t> Graph<VertexT, EdgeT>::GetNextEdges() const {
        return nextEdges_;
    }

    template <typename VertexT, typename EdgeT>
    std::span<const size_t> Graph<VertexT, EdgeT>::GetPrevEdges() const {
        return prevEdges_;
    }

    template <typename VertexT, typename EdgeT>
    void Graph<VertexT, EdgeT>::SetNextAndPrevEdges() {
        for (size_t i = verticesCount_, j = length_ - 1; i < length_; ++i, --j) {
            size_t tmp = indices_[i] - 1U;

            while (nextEdges_[tmp] != indices_[i] - 1U && nextEdges_[tmp] != 0U)
                tmp = nextEdges_[tmp];
            
            nextEdges_[tmp] = i;
            nextEdges_[i] = indices_[i] - 1U;

            tmp = indices_[j] - 1U;

            while (prevEdges_[tmp] != indices_[j] - 1U && prevEdges_[tmp] != 0U)
                tmp = prevEdges_[tmp];
            
            prevEdges_[tmp] = j;
            prevEdges_[j] = indices_[j] - 1U;
        }
    }

    template <typename VertexT, typename EdgeT>
    void Graph<VertexT, EdgeT>::AddVertexToTable(size_t vertexIndex) {
        indices_[vertexIndex  - 1U] = 0U;
        //std::cerr << vertexIndex << " " << std::endl;
    }

    template <typename VertexT, typename EdgeT>
    void Graph<VertexT, EdgeT>::AddVertexToTable(size_t vertexIndex, const VertexT &val) {
        indices_[vertexIndex  - 1U] = 0U;
        vertices_[vertexIndex  - 1U] = val;
        //std::cerr << vertexIndex << " " << std::endl;
    }

    template <typename VertexT, typename EdgeT>
    void Graph<VertexT, EdgeT>::AddEdgeToTable(std::pair<size_t, size_t> &&verticesPair) {
        indices_[verticesCount_ + curEdgesCount_] = verticesPair.first;
        indices_[verticesCount_ + curEdgesCount_ + 1U] = verticesPair.second;
        curEdgesCount_ += 2;
    }

    template <typename VertexT, typename EdgeT>
    void Graph<VertexT, EdgeT>::FillTable() {
        verticesCount_ += (verticesCount_ % 2);
        length_ = verticesCount_ + 2 * edgesCount_;
        
        indices_.resize(length_);
        vertices_.resize(verticesCount_, Vertex(VertexT()));
        nextEdges_.resize(length_);
        prevEdges_.resize(length_);

        AddVertexToTable(verticesCount_);
        for (size_t i = 0; i < verticesCount_; ++i) {
            nextEdges_[i] = i;
            prevEdges_[i] = i;
        }
        
        for (auto &&edge : edges_) {
            size_t fVertexIndex = edge.GetFirstVertex(), \
            sVertexIndex = edge.GetSecondVertex();
            AddVertexToTable(fVertexIndex);
            AddVertexToTable(sVertexIndex);
            AddEdgeToTable({fVertexIndex, sVertexIndex});
        }

        assert(curEdgesCount_ == 2 * edgesCount_);
        assert(edges_.size() == edgesCount_);
        assert(vertices_.size() == verticesCount_);

        SetNextAndPrevEdges();

        assert(indices_.size() == prevEdges_.size());
        assert(indices_.size() == nextEdges_.size());
        assert(indices_.size() == length_);
    }

    template <typename GraphT, typename Visitor>
    std::set<typename GraphT::VertexIndex> DoDepthFirstSearch(const GraphT &graph, typename GraphT::VertexIndex startVertex, Visitor &visitor) {
        std::vector<details::Color> colors{graph.GetVerticesCount(), details::Color::White};
        std::stack<typename GraphT::VertexIndex> stack;
        std::set<typename GraphT::VertexIndex> marked;

        stack.push(startVertex);
        colors[startVertex] = details::Color::Gray;

        while (!stack.empty()) {
            auto &&current = stack.top();
            stack.pop();
            marked.insert(current);

            if (colors[current] == details::Color::Gray) {
                colors[current] = details::Color::Black;
                visitor.VisitVertex(current, graph);
                
                for (auto &&neighbour : graph.GetNeighboringVertices(current)) {
                    if (colors[neighbour - 1U] == details::Color::White) {
                        stack.push(neighbour - 1U);
                        colors[neighbour - 1U] = details::Color::Gray;
                    }
                }
            }
        }

        return marked;
    }

    template <typename GraphT, typename Visitor>
    std::set<typename GraphT::VertexIndex> DoBreadthFirstSearch(const GraphT &graph, typename GraphT::VertexIndex startVertex, Visitor &visitor) {
        std::vector<details::Color> colors{graph.GetVerticesCount(), details::Color::White};    
        std::queue<typename GraphT::VertexIndex> queue;
        std::set<typename GraphT::VertexIndex> marked;

        queue.push(startVertex);

        while (!queue.empty()) {
            auto &&current = queue.front();
            queue.pop();
            colors[current] = details::Color::Black;
            marked.insert(current);

            visitor.VisitVertex(current, graph);
            for (auto &&neighbour : graph.GetNeighboringVertices(current)) {
                if (colors[neighbour - 1U] == details::Color::White)
                    queue.push(neighbour - 1U);
            }
        }

        return marked;
    }
}; // namespace graph