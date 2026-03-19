#pragma once 
#include <vector> 
#include <algorithm>

struct Lane 
{ 
    int capacity = 3; 
    int currentLoad = 0; 
    double currentSpeed = 0.0; 
    Lane(int cap = 3) : capacity(cap), currentLoad(0), currentSpeed(0.0) {} 
};

struct Edge
{
    int from;
    int to;

    double length;
    double baseSpeed;

    // NEW: cached average speed for routing
    double currentSpeed = 0.0;

    std::vector<Lane> lanes;

    bool isBlocked = false;

    // Metrics
    int maxLoadObserved = 0;
    double cumulativeLoad = 0.0;
    int samples = 0;

    Edge(int f,
        int t,
        double len,
        double speed,
        int laneCount = 2,
        int laneCapacity = 3)
        : from(f),
        to(t),
        length(len),
        baseSpeed(speed),
        currentSpeed(speed)
    {
        lanes.reserve(laneCount);
        for (int i = 0; i < laneCount; ++i)
            lanes.emplace_back(laneCapacity);
    }

    int getTotalLoad() const
    {
        int total = 0;
        for (const auto& lane : lanes)
            total += lane.currentLoad;
        return total;
    }

    int getTotalCapacity() const
    {
        int total = 0;
        for (const auto& lane : lanes)
            total += lane.capacity;
        return total;
    }

    double getAverageSpeed() const
    {
        if (lanes.empty())
            return baseSpeed;

        double total = 0.0;
        for (const auto& lane : lanes)
            total += lane.currentSpeed;

        return total / lanes.size();
    }

    void resetLoads()
    {
        for (auto& lane : lanes)
            lane.currentLoad = 0;
    }
};