#include "TrafficEngine.h"
#include <iostream>
#include <string>
#include "ScenarioRunner.h"

int main(int argc, char* argv[])
{
    bool runSweep = false;

    // ---------------------------------------
    // Command-line argument parsing
    // ---------------------------------------
    if (argc > 1)
    {
        std::string mode = argv[1];

        if (mode == "sweep")
            runSweep = true;
    }

    if (runSweep)
    {
        // ========================================
        // DEMAND SWEEP MODE
        // ========================================

        ScenarioRunner runner;

        runner.AddDemandLevel(0.30);
        runner.AddDemandLevel(0.20);
        runner.AddDemandLevel(0.10);
        runner.AddDemandLevel(0.05);
        runner.AddDemandLevel(0.02);

        runner.RunSweep(
            60.0,  // warmup time
            60.0,  // measurement time
            0.1);  // deltaTime

        std::cout << "\n=== DEMAND SWEEP RESULTS ===\n";

        for (const auto& r : runner.GetResults())
        {
            std::cout << "Spawn Interval: "
                << r.spawnInterval
                << " | Capacity: "
                << r.maxFlow
                << " | Critical Density: "
                << r.criticalDensity
                << "\n";
        }
    }
    else
    {
        // ========================================
        // SINGLE RUN MODE
        // ========================================

        TrafficEngine engine;
        engine.Initialize();

        const double deltaTime = 0.2;
        const double totalTime = 120.0;

        double simTime = 0.0;

        while (simTime < totalTime)
        {
            engine.Tick(deltaTime);

            Metrics m = engine.GetMetrics();

            std::cout << "Time: " << simTime
                << " | Active: "
                << m.activeVehicles
                << "\n";

            simTime += deltaTime;
        }

        Metrics m = engine.GetMetrics();

        std::cout << "\n=== METRICS ===\n";
        std::cout << "Completed Vehicles: "
            << m.completedVehicles << "\n";

        std::cout << "Average Travel Time: "
            << m.averageTravelTime << "\n";

        std::cout << "Throughput (veh/sec): "
            << m.throughput << "\n";

        std::cout << "Congestion Detected: "
            << (m.congestionDetected ? "YES" : "NO") << "\n";

        engine.PrintFundamentalDiagramSummary();
    }

    return 0;
}