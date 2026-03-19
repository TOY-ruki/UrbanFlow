#pragma once

#include <vector>
#include "Graph.h"

class Vehicle
{
public:
    enum class State
    {
        MOVING,
        WAITING,
        FINISHED
    };

    Vehicle(int id,
        const std::vector<int>& initialPath,
        const Graph* graph,
        int destinationNode,
        double spawnTime);

    void update(double deltaTime, double currentTime);

    // =============================
    // State
    // =============================
    bool hasFinished() const { return state == State::FINISHED; }
    bool isWaiting()  const { return state == State::WAITING; }

    void setWaiting(bool wait);
    void advanceToNextEdge(double currentTime);

    // =============================
    // Routing / Position
    // =============================
    int getCurrentFrom() const;
    int getCurrentTo() const;

    void setSpeed(double s);

    double getX() const { return posX; }
    double getY() const { return posY; }

    double getDistanceOnEdge() const { return distanceOnEdge; }
    double getCurrentSpeed() const { return currentSpeed; }

    int getId() const { return id; }

    double getTravelTime() const;
    int getRerouteCount() const { return rerouteCount; }

    // =============================
    // Lane System
    // =============================
    int getCurrentLaneIndex() const { return currentLaneIndex; }
    void assignLane(int laneIndex);

    // leader detection
    double computeIDMAcceleration(
        double currentSpeed,
        double desiredSpeed,
        double gapToLeader,
        double leaderSpeed) const;

private:

    // =============================
    // Identity / Graph
    // =============================
    int id;
    const Graph* graph;

    std::vector<int> path;
    size_t currentIndex = 0;
    int destinationNode;

    // =============================
    // Lane Tracking
    // =============================
    int currentLaneIndex = -1;
    double laneChangeTimer = 0.0;
    double laneChangeCooldown = 0.0;
    double laneChangeInterval = 1.0;

    // =============================
    // Movement
    // =============================
    double distanceOnEdge = 0.0;
    double baseSpeed = 5.0;
    double currentSpeed = 5.0;

    double posX = 0.0;
    double posY = 0.0;

    State state = State::MOVING;

    // ==============================
    // Driver Personality
    // ==============================

    double aggressiveness = 1.0;
    double politenessFactor = 0.3;
    double minImprovement = 1.0;
    double personalRerouteTolerance = 0.95;

    double reactionTime = 0.5;
    double accelerationRate = 2.5;
    double decelerationRate = 4.0;

    // ==============================
    // IDM Car-Following Model
    // ==============================

    double desiredTimeHeadway = 1.5;
    double minimumGap = 2.0;
    double maxAcceleration = 2.0;
    double comfortableDeceleration = 3.0;
    double deltaExponent = 4.0;

    // =============================
    // Rerouting
    // =============================
    double waitingTimer = 0.0;
    double rerouteInterval = 1.0;
    double rerouteTolerance = 0.95;

    double movingRerouteTimer = 0.0;
    double movingRerouteInterval = 1.5;

    int rerouteCount = 0;

    // =============================
    // Metrics
    // =============================
    double spawnTime = 0.0;
    double arrivalTime = -1.0;

private:

    void tryWaitingReroute(double deltaTime);
    void tryPredictiveReroute(double deltaTime);

    double estimateRemainingTime(
        const std::vector<int>& p,
        size_t startIndex) const;

    void updatePosition(double deltaTime, double currentTime);
};