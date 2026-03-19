#pragma once

#include <unordered_map>
#include <vector>
#include "Node.h"
#include "Edge.h"

class Graph
{
public:
    void addNode(int id, double x, double y);
    void addEdge(int from, int to, double speedLimit);
    void addBidirectionalEdge(int a, int b, double speedLimit);

    const std::unordered_map<int, Node>& GetNodes() const;

    const std::unordered_map<int, std::vector<Edge>>& GetAdjacency() const;
    std::unordered_map<int, std::vector<Edge>>& GetAdjacencyMutable();

    std::vector<int> findPath(int start, int goal) const;

private:
    std::unordered_map<int, Node> nodes;
    std::unordered_map<int, std::vector<Edge>> adjacency;
};






























