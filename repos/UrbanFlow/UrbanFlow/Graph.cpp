#include "Graph.h"
#include <cmath>
#include <queue>
#include <limits>
#include <algorithm>

// ======================================================
// Node / Edge Creation
// ======================================================

void Graph::addNode(int id, double x, double y)
{
    nodes.emplace(id, Node(id, x, y));
}

void Graph::addEdge(int from, int to, double speedLimit)
{
    const Node& a = nodes.at(from);
    const Node& b = nodes.at(to);

    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double length = std::sqrt(dx * dx + dy * dy);

    // Default: 2 lanes, 3 vehicles per lane
    adjacency[from].emplace_back(from, to, length, speedLimit, 2, 3);
}

void Graph::addBidirectionalEdge(int a, int b, double speedLimit)
{
    addEdge(a, b, speedLimit);
    addEdge(b, a, speedLimit);
}

// ======================================================
// Accessors
// ======================================================

const std::unordered_map<int, Node>& Graph::GetNodes() const
{
    return nodes;
}

const std::unordered_map<int, std::vector<Edge>>& Graph::GetAdjacency() const
{
    return adjacency;
}

std::unordered_map<int, std::vector<Edge>>& Graph::GetAdjacencyMutable()
{
    return adjacency;
}

// ======================================================
// A* Pathfinding (Multi-Lane Travel Time Based)
// ======================================================

std::vector<int> Graph::findPath(int start, int goal) const
{
    struct NodeRecord
    {
        int node;
        double fScore;

        bool operator>(const NodeRecord& other) const
        {
            return fScore > other.fScore;
        }
    };

    std::priority_queue<
        NodeRecord,
        std::vector<NodeRecord>,
        std::greater<NodeRecord>> openSet;

    std::unordered_map<int, double> gScore;
    std::unordered_map<int, int> cameFrom;

    for (const auto& pair : nodes)
        gScore[pair.first] = std::numeric_limits<double>::infinity();

    gScore[start] = 0.0;

    auto heuristic = [&](int a, int b)
        {
            const Node& na = nodes.at(a);
            const Node& nb = nodes.at(b);

            double dx = na.x - nb.x;
            double dy = na.y - nb.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            constexpr double freeFlowSpeed = 50.0;
            return distance / freeFlowSpeed;
        };

    openSet.push({ start, heuristic(start, goal) });

    while (!openSet.empty())
    {
        int current = openSet.top().node;
        openSet.pop();

        if (current == goal)
        {
            std::vector<int> path;

            while (cameFrom.find(current) != cameFrom.end())
            {
                path.push_back(current);
                current = cameFrom[current];
            }

            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        auto adjIt = adjacency.find(current);
        if (adjIt == adjacency.end())
            continue;

        for (const Edge& edge : adjIt->second)
        {
            if (edge.isBlocked)
                continue;

            double speed = std::max(0.1, edge.getAverageSpeed());
            double travelTime = edge.length / speed;

            double tentativeG = gScore[current] + travelTime;

            if (tentativeG < gScore[edge.to])
            {
                cameFrom[edge.to] = current;
                gScore[edge.to] = tentativeG;

                double fScore = tentativeG + heuristic(edge.to, goal);
                openSet.push({ edge.to, fScore });
            }
        }
    }

    return {};
}