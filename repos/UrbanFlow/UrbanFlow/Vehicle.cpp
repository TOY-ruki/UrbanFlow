#include "Vehicle.h"
#include <cmath>
#include <algorithm>
#include <random>

// ======================================================
// Constructor
// ======================================================

Vehicle::Vehicle(int id_,
    const std::vector<int>& initialPath,
    const Graph* graph_,
    int destination_,
    double spawnTime_)
    : id(id_),
    graph(graph_),
    path(initialPath),
    destinationNode(destination_),
    spawnTime(spawnTime_)
{
    if (!path.empty())
    {
        const auto& nodes = graph->GetNodes();
        posX = nodes.at(path[0]).x;
        posY = nodes.at(path[0]).y;
    }

    // ==========================================
    // Driver Personality Randomization
    // ==========================================

    std::mt19937 rng(id_ * 73856093);   // deterministic per vehicle
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    double profile = dist(rng);

    if (profile < 0.2)
    {
        // Aggressive
        aggressiveness = 1.2;
        politenessFactor = 0.1;
        minImprovement = 0.5;
        rerouteTolerance = 0.90;
        movingRerouteInterval = 1.0;
    }
    else if (profile < 0.8)
    {
        // Normal
        aggressiveness = 1.0;
        politenessFactor = 0.3;
        minImprovement = 1.0;
        rerouteTolerance = 0.95;
        movingRerouteInterval = 1.5;
    }
    else
    {
        // Conservative
        aggressiveness = 0.85;
        politenessFactor = 0.6;
        minImprovement = 1.5;
        rerouteTolerance = 0.98;
        movingRerouteInterval = 2.0;
    }
    std::uniform_real_distribution<double> reactionDist(0.3, 1.0);
    reactionTime = reactionDist(rng);

    // Aggressive drivers react faster
    reactionTime *= (1.2 - aggressiveness);

    // Acceleration / braking personality
    accelerationRate = 2.0 * aggressiveness;
    decelerationRate = 3.5 * aggressiveness;
}

// ======================================================
// Main Update
// ======================================================

void Vehicle::update(double deltaTime, double currentTime)
{
    if (state == State::FINISHED)
        return;

    if (state == State::WAITING)
    {
        tryWaitingReroute(deltaTime);
        return;
    }

    tryPredictiveReroute(deltaTime);
    updatePosition(deltaTime, currentTime);
}

// ======================================================
// Movement Logic
// ======================================================

void Vehicle::updatePosition(double deltaTime, double currentTime)
{
    if (currentIndex >= path.size() - 1)
    {
        state = State::FINISHED;
        arrivalTime = currentTime;
        return;
    }

    int from = path[currentIndex];
    int to = path[currentIndex + 1];

    const auto& adj = graph->GetAdjacency();
    const auto& edges = adj.at(from);

    const Edge* currentEdge = nullptr;

    for (const auto& e : edges)
    {
        if (e.to == to)
        {
            currentEdge = &e;
            break;
        }
    }

    if (!currentEdge || currentEdge->isBlocked)
    {
        state = State::WAITING;
        return;
    }

    // ==================================================
    // Ensure lane assigned
    // ==================================================

    if (currentLaneIndex < 0 ||
        currentLaneIndex >= static_cast<int>(currentEdge->lanes.size()))
    {
        state = State::WAITING;
        return;
    }

    // ==================================================
    // Lane-based speed
    // ==================================================

    const Lane& lane =
        currentEdge->lanes[currentLaneIndex];

    double laneSpeed = std::max(0.1, lane.currentSpeed);
    
    double desiredSpeed = laneSpeed * aggressiveness;

    // ----------------------------------------
    // IDM acceleration (no leader yet)
    // ----------------------------------------

    double gapToLeader = 1000.0;       // assume free road
    double leaderSpeed = desiredSpeed; // no leader ahead

    double accel = computeIDMAcceleration(
        currentSpeed,
        desiredSpeed,
        gapToLeader,
        leaderSpeed);

    // Integrate speed
    currentSpeed += accel * deltaTime;

    // Prevent negative speeds
    currentSpeed = std::max(0.0, currentSpeed);


    // ==================================================
    // Move along edge
    // ==================================================

    const auto& nodes = graph->GetNodes();

    double x1 = nodes.at(from).x;
    double y1 = nodes.at(from).y;
    double x2 = nodes.at(to).x;
    double y2 = nodes.at(to).y;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double edgeLength = std::sqrt(dx * dx + dy * dy);

    if (edgeLength < 1e-6)
    {
        currentIndex++;
        return;
    }

    distanceOnEdge += currentSpeed * deltaTime;

    if (distanceOnEdge >= edgeLength)
    {
        distanceOnEdge = edgeLength;
        state = State::WAITING;
        return;
    }

    double t = distanceOnEdge / edgeLength;
    posX = x1 + t * dx;
    posY = y1 + t * dy;

    // ==================================================
    // Lane Change Timer
    // ==================================================

    laneChangeTimer += deltaTime;
}

// ======================================================
// Waiting Reroute
// ======================================================

void Vehicle::tryWaitingReroute(double deltaTime)
{
    waitingTimer += deltaTime;

    if (waitingTimer < rerouteInterval)
        return;

    waitingTimer = 0.0;

    int currentNode = path[currentIndex];

    std::vector<int> newPath =
        graph->findPath(currentNode, destinationNode);

    if (newPath.empty())
        return;

    double oldTime = estimateRemainingTime(path, currentIndex);
    double newTime = estimateRemainingTime(newPath, 0);

    if (newTime < oldTime * rerouteTolerance)
    {
        path = newPath;
        currentIndex = 0;
        distanceOnEdge = 0.0;
        currentLaneIndex = -1;
        rerouteCount++;
    }
}

// ======================================================
// Predictive Reroute
// ======================================================

void Vehicle::tryPredictiveReroute(double deltaTime)
{
    movingRerouteTimer += deltaTime;

    if (movingRerouteTimer < movingRerouteInterval)
        return;

    movingRerouteTimer = 0.0;

    int currentNode = path[currentIndex];

    std::vector<int> newPath =
        graph->findPath(currentNode, destinationNode);

    if (newPath.empty())
        return;

    double oldTime = estimateRemainingTime(path, currentIndex);
    double newTime = estimateRemainingTime(newPath, 0);

    if (newTime < oldTime * rerouteTolerance)
    {
        path = newPath;
        currentIndex = 0;
        distanceOnEdge = 0.0;
        currentLaneIndex = -1;
        rerouteCount++;
    }
}

// ======================================================
// Lane Assignment
// ======================================================

void Vehicle::assignLane(int laneIndex)
{
    currentLaneIndex = laneIndex;
}

// ======================================================
// Remaining Time Estimation
// ======================================================

double Vehicle::estimateRemainingTime(
    const std::vector<int>& p,
    size_t startIndex) const
{
    double total = 0.0;

    for (size_t i = startIndex; i < p.size() - 1; i++)
    {
        const auto& edges = graph->GetAdjacency().at(p[i]);

        for (const auto& edge : edges)
        {
            if (edge.to == p[i + 1])
            {
                double speed = std::max(0.1, edge.getAverageSpeed());
                total += edge.length / speed;
                break;
            }
        }
    }

    return total;
}

// ======================================================
// State Management
// ======================================================

void Vehicle::setWaiting(bool wait)
{
    state = wait ? State::WAITING : State::MOVING;
}

void Vehicle::advanceToNextEdge(double currentTime)
{
    currentIndex++;
    distanceOnEdge = 0.0;
    currentLaneIndex = -1;

    if (currentIndex >= path.size() - 1)
    {
        state = State::FINISHED;
        arrivalTime = currentTime;
    }
}

// ======================================================
// Accessors
// ======================================================

int Vehicle::getCurrentFrom() const
{
    if (state == State::FINISHED ||
        currentIndex >= path.size() - 1)
        return -1;

    return path[currentIndex];
}

int Vehicle::getCurrentTo() const
{
    if (state == State::FINISHED ||
        currentIndex >= path.size() - 1)
        return -1;

    return path[currentIndex + 1];
}

void Vehicle::setSpeed(double s)
{
    currentSpeed = s;
}

double Vehicle::getTravelTime() const
{
    if (arrivalTime < 0.0)
        return 0.0;

    return arrivalTime - spawnTime;
}
//==================
//IDM implementation
//==================
double Vehicle::computeIDMAcceleration(
    double v,
    double v0,
    double gap,
    double vLeader) const
{
    if (gap < 0.1)
        gap = 0.1;

    double dv = v - vLeader;

    double sStar =
        minimumGap +
        v * desiredTimeHeadway +
        (v * dv) /
        (2.0 * std::sqrt(maxAcceleration * comfortableDeceleration));

    double accel =
        maxAcceleration *
        (1.0 - std::pow(v / v0, deltaExponent)
            - std::pow(sStar / gap, 2.0));

    return accel;
}