#include <iostream>
#include <vector>
#include <string>
#include <set>
#include <chrono>

#include "../Algorithms/Dijkstra/Dijkstra.h"

template<typename GRAPH, bool DEBUG = false>
int main() {
    auto startTime = std::chrono::high_resolution_clock::now();

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
    for (const auto& input : inputs) {
        int source = input.source;
        int target = input.target;
        const std::vector<int>& expectedPath = input.expectedPath;
        int expectedLength = input.expectedLength;

        dijkstra.run(source, target); 

        std::cout << "Source: " << source << ", Target: " << target << std::endl;
        std::cout << "Shortest Path:";
        auto path = dijkstra.getPath(target); 
        for (int vertex : path) {
            std::cout << " " << vertex;
        }
        std::cout << std::endl;
        
        int pathLength = dijkstra.getDistance(target); // Call member function on the dijkstra instance
        std::cout << "Shortest path length: " << pathLength << std::endl;
        
        if (path == expectedPath && pathLength == expectedLength) {
            std::cout << "Test PASSED" << std::endl;
        } else {
            std::cout << "Test FAILED" << std::endl;
        }
        
        std::cout << std::endl;
    }
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime);
    cout << "Total runtime: " << duration.count() << endl;
    return 0;
}
