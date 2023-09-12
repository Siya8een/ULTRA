#include <chrono>
#include <iostream>
#include <vector>
#include <string>
#include <set>

#include "../../Helpers/Meta.h"
#include "../../Helpers/Types.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/Vector/Vector.h"

#include "../../DataStructures/Container/ExternalKHeap.h"
#include "../../DataStructures/Container/Set.h"
#include "../../DataStructures/Attributes/AttributeNames.h"

#include "../Algorithms/Dijkstra/Dijkstra.h"

template<typename GRAPH, bool DEBUG = false>
class Dijkstra {

public:
    using Graph = GRAPH;
    static constexpr bool Debug = DEBUG;
    using Type = Dijkstra<Graph, Debug>;

private:
    std::vector<VertexLabel> backwardLabel;

    public:
    // Constructor for backwardLabel
    Dijkstra(const GRAPH& graph, const std::vector<int>& weight) :
        graph(graph),
        weight(weight),
        Q(graph.numVertices()),
        label(graph.numVertices()),
        backwardLabel(graph.numVertices()), // Initialize backwardLabel
        timeStamp(0),
        settleCount(0) {
    }

public:
    struct VertexLabel : public ExternalKHeapElement {
        VertexLabel() : ExternalKHeapElement(), distance(intMax), parent(noVertex), timeStamp(-1) {}
        inline void reset(int time) {
            distance = intMax;
            parent = noVertex;
            timeStamp = time;
        }
        inline bool hasSmallerKey(const VertexLabel* other) const {
            return distance < other->distance;
        }

        int distance;
        Vertex parent;
        int timeStamp;
    };

public:
    Dijkstra(const GRAPH& graph, const std::vector<int>& weight) :
        graph(graph),
        weight(weight),
        Q(graph.numVertices()),
        label(graph.numVertices()),
        timeStamp(0),
        settleCount(0) {
    }

    Dijkstra(const GRAPH& graph) :
        Dijkstra(graph, graph[TravelTime]){
    }

    template<AttributeNameType ATTRIBUTE_NAME>
    Dijkstra(const GRAPH& graph, const AttributeNameWrapper<ATTRIBUTE_NAME> weight) :
        Dijkstra(graph, graph[weight]){
    }

    Dijkstra(const GRAPH&&, const std::vector<int>&) = delete;
    Dijkstra(const GRAPH&, const std::vector<int>&&) = delete;
    Dijkstra(const GRAPH&&) = delete;

    template<AttributeNameType ATTRIBUTE_NAME>
    Dijkstra(const GRAPH&&, const AttributeNameWrapper<ATTRIBUTE_NAME>) = delete;

   template<typename SETTLE = NO_OPERATION, typename STOP = NO_OPERATION, typename PRUNE_EDGE = NO_OPERATION>
    inline void run(const Vertex forwardTarget, const Vertex backwardTarget, const SETTLE& settle = NoOperation, const STOP& stop = NoOperation, const PRUNE_EDGE& pruneEdge = NoOperation) noexcept {
        clear();
        addSource(forwardTarget);
        addSourceBackward(backwardTarget); // Add the backward target
        bool forwardFinished = false;
        bool backwardFinished = false;

        while (!Q.empty() && !(forwardFinished && backwardFinished)) {
            if (stop()) break;

            if (!forwardFinished) {
                VertexLabel* uLabel = Q.extractFront();
                const Vertex u = Vertex(uLabel - &(label[0]));

                if (u == forwardTarget) {
                    forwardFinished = true;
                    continue; // Skip forward target
                }

                for (const Edge edge : graph.edgesFrom(u)) {
                    // Forward search logic here
                }

                settle(u);
                if constexpr (Debug) settleCount++;
            }

            if (!backwardFinished) {
                VertexLabel* uLabel = backwardQ.extractFront();
                const Vertex u = Vertex(uLabel - &(backwardLabel[0]));

                if (u == backwardTarget) {
                    backwardFinished = true;
                    continue; // Skip backward target
                }

                for (const Edge edge : graph.edgesTo(u)) {
                    // Backward search logic here
                }

                settleBackward(u);
                if constexpr (Debug) settleCount++; // You might need separate counters for forward and backward
            }
        }

        if constexpr (Debug) {
            // Debug output for both forward and backward searches
        }
    }
    inline void addSourceBackward(const Vertex source, const int distance = 0) noexcept {
        VertexLabel& sourceLabel = getBackwardLabel(source);
        sourceLabel.distance = distance;
        backwardQ.update(&sourceLabel);
    }


private:
    //  function to get the backward label for a vertex
    inline VertexLabel& getBackwardLabel(const Vertex vertex) noexcept {
        VertexLabel& result = backwardLabel[vertex];
        if (result.timeStamp != timeStamp) result.reset(timeStamp);
        return result;
    }

    inline void clear() noexcept {
        if constexpr (Debug) {
            timer.restart();
            settleCount = 0;
        }
        Q.clear();
        timeStamp++;
    }

    inline void addSource(const Vertex source, const int distance = 0) noexcept {
        VertexLabel& sourceLabel = getLabel(source);
        sourceLabel.distance = distance;
        Q.update(&sourceLabel);
    }

    inline void run() noexcept {
        run(noVertex, NoOperation, NoOperation, NoOperation);
    }

    template<typename SETTLE, typename STOP = NO_OPERATION, typename PRUNE_EDGE = NO_OPERATION, typename = decltype(std::declval<SETTLE>()(std::declval<Vertex>()))>
    inline void run(const Vertex target, const SETTLE& settle, const STOP& stop = NoOperation, const PRUNE_EDGE& pruneEdge = NoOperation) noexcept {
        while(!Q.empty()) {
            if (stop()) break;
            VertexLabel* uLabel = Q.extractFront();
            const Vertex u = Vertex(uLabel - &(label[0]));
            if (u == target) break;
            for (const Edge edge : graph.edgesFrom(u)) {
                const Vertex v = graph.get(ToVertex, edge);
                VertexLabel& vLabel = getLabel(v);
                if (pruneEdge(u, edge)) continue;
                const int distance = uLabel->distance + weight[edge];
                if (vLabel.distance > distance) {
                    vLabel.distance = distance;
                    vLabel.parent = u;
                    Q.update(&vLabel);
                }
            }
            settle(u);
            if constexpr (Debug) settleCount++;
        }
        if constexpr (Debug) {
            std::cout << "Settled Vertices = " << String::prettyInt(settleCount) << std::endl;
            std::cout << "Time = " << String::msToString(timer.elapsedMilliseconds()) << std::endl;
        }
    }

    inline bool reachable(const Vertex vertex) const noexcept {
        return label[vertex].timeStamp == timeStamp;
    }

    inline bool visited(const Vertex vertex) const noexcept {
        return label[vertex].timeStamp == timeStamp;
    }

    inline int getDistance(const Vertex vertex) const noexcept {
        if (visited(vertex)) return label[vertex].distance;
        return -1;
    }

    inline Vertex getParent(const Vertex vertex) const noexcept {
        if (visited(vertex)) return label[vertex].parent;
        return noVertex;
    }

    inline std::set<Vertex> getChildren(const Vertex vertex) const noexcept {
        if (visited(vertex)) {
            std::set<Vertex> children;
            for (Vertex child : graph.outgoingNeighbors(vertex)) {
                if (label[child].parent == vertex) {
                    children.insert(child);
                }
            }
            return children;
        }
        return std::set<Vertex>();
    }

    inline Vertex getQFront() const noexcept {
        if (Q.empty()) return noVertex;
        return Vertex(Q.front() - &(label[0]));
    }

    inline std::vector<Vertex> getReversePath(const Vertex to) const noexcept {
        std::vector<Vertex> path;
        if (!visited(to)) return path;
        path.push_back(to);
        while (label[path.back()].parent != noVertex) {
            path.push_back(label[path.back()].parent);
        }
        return path;
    }

    inline std::vector<Vertex> getPath(const Vertex to) const noexcept {
        return Vector::reverse(getReversePath(to));
    }

    inline int getSettleCount() const noexcept {
        return settleCount;
    }

private:
    inline VertexLabel& getLabel(const Vertex vertex) noexcept {
        VertexLabel& result = label[vertex];
        if (result.timeStamp != timeStamp) result.reset(timeStamp);
        return result;
    }

private:
    const GRAPH& graph;
    const std::vector<int>& weight;

    ExternalKHeap<2, VertexLabel> Q;

    std::vector<VertexLabel> label;
    int timeStamp;

    int settleCount;
    Timer timer;

};

template<typename GRAPH, bool DEBUG = false>

int main() {
    auto startTime = std::chrono::high_resolution_clock::now();
    srand(static_cast<unsigned int>(time(nullptr)));

    const int numTestCases = 200;

    GRAPH graph;
    Dijkstra<GRAPH, false> dijkstra(graph);

    struct Input {
        int source;
        int target;
        std::vector<int> expectedPath;
        int expectedLength;
    };

    std::vector<Input> inputs;

    std::ifstream inputFile("florida.gr");
    if (!inputFile.is_open()) {
        std::cerr << "Failed to open 'florida.gr' file!" << std::endl;
        return 1;
    }

    std::string line;
    while (std::getline(inputFile, line)) {
        int source, target, expectedLength;
        std::vector<int> expectedPath;

        std::istringstream iss(line);
        if (!(iss >> source >> target)) {
            std::cerr << "Error reading input data from file!" << std::endl;
            return 1;
        }

        int pathValue;
        while (iss >> pathValue) {
            expectedPath.push_back(pathValue);
        }

        inputs.push_back({source, target, expectedPath, expectedLength});
    }

    inputFile.close();

    for (int i = 0; i < numTestCases; ++i) {
        int source = testCases[i].source;
        int target = testCases[i].target;

        dijkstra.run(source, target);

    }
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime);
    cout << "Total runtime: " << duration.count() << endl;

    return 0;
}

