#include "ScenarioRunner.h"

ScenarioRunner::ScenarioRunner()
{
    engine.Initialize();
}

void ScenarioRunner::AddDemandLevel(double spawnInterval)
{
    demandLevels.push_back(spawnInterval);
}

void ScenarioRunner::RunSweep(
    double warmupTime,
    double measurementTime,
    double deltaTime)
{
    results.clear();

    for (double interval : demandLevels)
    {
        engine.Reset();
        engine.SetSpawnInterval(interval);

        double simTime = 0.0;

        // -------------------------
        // Warmup Phase
        // -------------------------
        while (simTime < warmupTime)
        {
            engine.Tick(deltaTime);
            simTime += deltaTime;
        }

        // -------------------------
        // Measurement Phase
        // -------------------------
        simTime = 0.0;

        while (simTime < measurementTime)
        {
            engine.Tick(deltaTime);
            simTime += deltaTime;
        }

        GlobalFDData globalFD = engine.GetGlobalFDData();

        SweepResult result;
        result.spawnInterval = interval;
        result.maxFlow = globalFD.maxObservedFlow;
        result.criticalDensity = globalFD.criticalDensity;

        results.push_back(result);
    }
}

const std::vector<SweepResult>&
ScenarioRunner::GetResults() const
{
    return results;
}