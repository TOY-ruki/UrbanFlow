#pragma once

#include <vector>
#include "TrafficEngine.h"

struct SweepResult
{
    double spawnInterval;
    double maxFlow;
    double criticalDensity;
};

class ScenarioRunner
{
public:
    ScenarioRunner();

    void AddDemandLevel(double spawnInterval);

    void RunSweep(
        double warmupTime,
        double measurementTime,
        double deltaTime);

    const std::vector<SweepResult>& GetResults() const;

private:
    TrafficEngine engine;
    std::vector<double> demandLevels;
    std::vector<SweepResult> results;
};
