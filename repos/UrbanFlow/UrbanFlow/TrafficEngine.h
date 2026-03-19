#pragma once

#include <vector>
#include <unordered_map>
#include "Graph.h"
#include "Vehicle.h"

// ==============================
// Snapshot
// ==============================

struct VehicleSnapshot
{
    int id;
    double x;
    double y;
};

// ==============================
// Metrics
// ==============================

struct Metrics
{
    int activeVehicles = 0;
    int completedVehicles = 0;
    double averageTravelTime = 0.0;
    double throughput = 0.0;
    bool congestionDetected = false;
};

// ==============================
// Lane Key (For Leader Lookup)
// ==============================

struct LaneKey
{
    int from;
    int to;
    int laneIndex;

    bool operator==(const LaneKey& other) const
    {
        return from == other.from &&
            to == other.to &&
            laneIndex == other.laneIndex;
    }
};

struct LaneKeyHash
{
    std::size_t operator()(const LaneKey& k) const
    {
        return std::hash<int>()(k.from) ^
            (std::hash<int>()(k.to) << 1) ^
            (std::hash<int>()(k.laneIndex) << 2);
    }
};

// ==============================
// Edge Key (For FD Measurement)
// ==============================

struct EdgeKey
{
    int from;
    int to;

    bool operator==(const EdgeKey& other) const
    {
        return from == other.from &&
            to == other.to;
    }
};

struct EdgeKeyHash
{
    std::size_t operator()(const EdgeKey& k) const
    {
        return std::hash<int>()(k.from) ^
            (std::hash<int>()(k.to) << 1);
    }
};

// ==============================
// Fundamental Diagram Data (Per Edge)
// ==============================

struct EdgeFDData
{
    // Instantaneous values (last sampling window)
    double density = 0.0;       // vehicles per unit length
    double flow = 0.0;          // vehicles per second
    double averageSpeed = 0.0;  // space-mean speed

    // Accumulated statistics
    double densitySum = 0.0;
    double flowSum = 0.0;
    double speedSum = 0.0;
    int samples = 0;

    // Exit counter for flow measurement
    int exitCounter = 0;


    // ==========================================
    // Fundamental Diagram Curve Tracking
    // ==========================================

    std::vector<double> densityHistory;
    std::vector<double> flowHistory;
    std::vector<double> speedHistory;

    double maxObservedFlow = 0.0;
    double criticalDensity = 0.0;


};

// ==============================
// Fundamental Diagram Data (Global)
// ==============================

struct GlobalFDData
{
    double density = 0.0;
    double flow = 0.0;
    double averageSpeed = 0.0;

    double densitySum = 0.0;
    double flowSum = 0.0;
    double speedSum = 0.0;
    int samples = 0;

    // ==========================================
    // Global FD Curve Tracking
    // ==========================================

    std::vector<double> densityHistory;
    std::vector<double> flowHistory;
    std::vector<double> speedHistory;

    double maxObservedFlow = 0.0;
    double criticalDensity = 0.0;



};


// ==============================
// Traffic Engine
// ==============================

class TrafficEngine
{
public:
    TrafficEngine();

    void Initialize();
    void Tick(double deltaTime);

    std::vector<VehicleSnapshot> GetVehicleSnapshots() const;
    Metrics GetMetrics() const;

    // ==============================
    // Fundamental Diagram Access
    // ==============================

    GlobalFDData GetGlobalFDData() const;

    bool GetEdgeFDData(int from, int to, EdgeFDData& outData) const;


    // ==============================
    // Diagnostic Output
    // ==============================

    void PrintFundamentalDiagramSummary() const;


    // ==============================
    // Scenario Control
    // ==============================

    void SetSpawnInterval(double interval);
    void Reset();



private:

    // ==============================
    // Core Systems
    // ==============================

    void HandleIncidents();
    void SpawnVehicles();

    void ResetLaneLoads();
    void UpdateLaneLoads();
    void UpdateLaneSpeeds();

    void BuildLaneVehicleLists();     // NEW
    void ApplyLeaderIDM(double dt);   // NEW

    void HandleWaitingVehicles(double deltaTime);
    void UpdateVehicles(double deltaTime);
    void RemoveFinishedVehicles();
    void DetectCongestion(double deltaTime);

    // ==============================
    // Fundamental Diagram Processing
    // ==============================

    void UpdateFundamentalDiagram(double deltaTime);

    // ==============================
    // Simulation State
    // ==============================

    Graph graph;
    std::vector<Vehicle> vehicles;

    double currentTime = 0.0;
    double spawnTimer = 0.0;
    double spawnInterval = 0.05;

    int nextVehicleId = 1;
    int maxVehicles = 500;

    // Metrics
    int completedVehicles = 0;
    double totalTravelTime = 0.0;

   // ==============================
   // Fundamental Diagram System
   // ==============================

    std::unordered_map<
        EdgeKey,
        EdgeFDData,
        EdgeKeyHash> edgeFDMap;

    GlobalFDData globalFD;

    double fdSamplingInterval = 1.0;   // seconds
    double fdTimer = 0.0;

    // Incident control
    bool incidentActive = false;
    bool incidentCleared = false;
    double incidentStartTime = 20.0;
    double incidentEndTime = 40.0;

    // Congestion detection
    bool congestionDetected = false;
    double growthTimer = 0.0;

    // ==============================
    // Lane → Vehicles container
    // ==============================

    std::unordered_map<
        LaneKey,
        std::vector<Vehicle*>,
        LaneKeyHash> laneVehicleMap;
};