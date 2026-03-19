#include "TrafficEngine.h"
#include <algorithm>
#include <map>
#include <iostream>

// ======================================================
// Constructor
// ======================================================

TrafficEngine::TrafficEngine()
{
}

void TrafficEngine::SetSpawnInterval(double interval)
{
    spawnInterval = interval;
}

void TrafficEngine::Reset()
{
    vehicles.clear();

    currentTime = 0.0;
    spawnTimer = 0.0;
    nextVehicleId = 1;

    completedVehicles = 0;
    totalTravelTime = 0.0;

    incidentActive = false;
    incidentCleared = false;
    congestionDetected = false;
    growthTimer = 0.0;

    // Reset FD system
    edgeFDMap.clear();
    globalFD = GlobalFDData();
    fdTimer = 0.0;

    // Reset edge metrics
    for (auto& pair : graph.GetAdjacencyMutable())
    {
        for (auto& edge : pair.second)
        {
            edge.maxLoadObserved = 0;
            edge.cumulativeLoad = 0.0;
            edge.samples = 0;

            for (auto& lane : edge.lanes)
            {
                lane.currentLoad = 0;
                lane.currentSpeed = edge.baseSpeed;
            }
        }
    }
}

// ======================================================
// Initialize Network
// ======================================================

void TrafficEngine::Initialize()
{
    graph.addNode(1, 0, 0);
    graph.addNode(2, 10, 0);
    graph.addNode(3, 20, 0);
    graph.addNode(4, 10, 10);
    graph.addNode(5, 20, 10);
    graph.addNode(8, 30, 0);

    graph.addBidirectionalEdge(1, 2, 50);
    graph.addBidirectionalEdge(2, 3, 40);
    graph.addBidirectionalEdge(3, 8, 40);

    graph.addBidirectionalEdge(1, 4, 45);
    graph.addBidirectionalEdge(4, 5, 45);
    graph.addBidirectionalEdge(5, 8, 40);

    auto path = graph.findPath(1, 8);
    vehicles.emplace_back(nextVehicleId++, path, &graph, 8, currentTime);
}

// ======================================================
// Tick
// ======================================================

void TrafficEngine::Tick(double deltaTime)
{
    currentTime += deltaTime;
    spawnTimer += deltaTime;

    HandleIncidents();
    SpawnVehicles();
    ResetLaneLoads();
    UpdateLaneLoads();
    UpdateLaneSpeeds();
    HandleWaitingVehicles(deltaTime);
    UpdateVehicles(deltaTime);
    RemoveFinishedVehicles();
    DetectCongestion(deltaTime);
    UpdateFundamentalDiagram(deltaTime);
}

// ======================================================
// Incident Handling
// ======================================================

void TrafficEngine::HandleIncidents()
{
    if (!incidentActive && currentTime >= incidentStartTime)
    {
        auto& edges = graph.GetAdjacencyMutable().at(2);

        for (auto& e : edges)
            if (e.to == 3)
                e.isBlocked = true;

        incidentActive = true;
    }

    if (!incidentCleared && currentTime >= incidentEndTime)
    {
        auto& edges = graph.GetAdjacencyMutable().at(2);

        for (auto& e : edges)
            if (e.to == 3)
                e.isBlocked = false;

        incidentCleared = true;
    }
}

// ======================================================
// Spawning
// ======================================================

void TrafficEngine::SpawnVehicles()
{
    if (spawnTimer < spawnInterval)
        return;

    spawnTimer = 0.0;

    auto path = graph.findPath(1, 8);
    vehicles.emplace_back(nextVehicleId++, path, &graph, 8, currentTime);
}

// ======================================================
// Multi-Lane Load Reset
// ======================================================

void TrafficEngine::ResetLaneLoads()
{
    for (auto& pair : graph.GetAdjacencyMutable())
    {
        for (auto& edge : pair.second)
        {
            for (auto& lane : edge.lanes)
                lane.currentLoad = 0;
        }
    }
}

// ======================================================
// Assign Vehicles to Lanes
// ======================================================
void TrafficEngine::UpdateLaneLoads()
{
    for (auto& v : vehicles)
    {
        if (v.hasFinished() || v.isWaiting())
            continue;

        int from = v.getCurrentFrom();
        int to = v.getCurrentTo();
        if (from < 0 || to < 0)
            continue;

        auto& edges = graph.GetAdjacencyMutable().at(from);

        for (auto& edge : edges)
        {
            if (edge.to != to)
                continue;

            // ---------------------------------
            // INITIAL LANE ASSIGNMENT
            // ---------------------------------
            if (v.getCurrentLaneIndex() < 0)
            {
                int bestLane = 0;
                int minLoad = edge.lanes[0].currentLoad;

                for (int i = 1; i < edge.lanes.size(); ++i)
                {
                    if (edge.lanes[i].currentLoad < minLoad)
                    {
                        minLoad = edge.lanes[i].currentLoad;
                        bestLane = i;
                    }
                }

                v.assignLane(bestLane);
            }

            int currentLane = v.getCurrentLaneIndex();

            // ---------------------------------
            // MOBIL LANE CHANGE LOGIC
            // ---------------------------------
            for (int offset : { -1, 1 }) // check left/right
            {
                int candidateLane = currentLane + offset;

                if (candidateLane < 0 ||
                    candidateLane >= edge.lanes.size())
                    continue;

                auto& curr = edge.lanes[currentLane];
                auto& cand = edge.lanes[candidateLane];

                if (cand.currentLoad >= cand.capacity)
                    continue;

                double currSpeed = curr.currentSpeed;
                double candSpeed = cand.currentSpeed;

                double ownGain = candSpeed - currSpeed;

                if (ownGain < 1.0)
                    continue;

                double disadvantage = curr.currentSpeed - cand.currentSpeed;

                double mobilCriterion =
                    ownGain - 0.3 * disadvantage;

                if (mobilCriterion > 1.0)
                {
                    v.assignLane(candidateLane);
                    currentLane = candidateLane;
                    break;
                }
            }

            edge.lanes[currentLane].currentLoad++;
            break;
        }
    }
}

// ======================================================
// Lane Speed Update (Per Lane)
// ======================================================

void TrafficEngine::UpdateLaneSpeeds()
{
    for (auto& pair : graph.GetAdjacencyMutable())
    {
        for (auto& edge : pair.second)
        {
            int totalLoad = 0;
            int totalCap = 0;

            for (auto& lane : edge.lanes)
            {
                totalLoad += lane.currentLoad;
                totalCap += lane.capacity;
            }

            double rho = totalCap > 0
                ? static_cast<double>(totalLoad) / totalCap
                : 0.0;

            rho = std::min(1.0, rho);

            double speedFactor = std::max(0.25, 1.0 - rho * rho);

            for (auto& lane : edge.lanes)
                lane.currentSpeed = edge.baseSpeed * speedFactor;

            edge.maxLoadObserved =
                std::max(edge.maxLoadObserved, totalLoad);

            edge.cumulativeLoad += totalLoad;
            edge.samples++;
        }
    }
}

// ======================================================
// Waiting Vehicles
// ======================================================

void TrafficEngine::HandleWaitingVehicles(double deltaTime)
{
    for (auto& v : vehicles)
    {
        if (!v.isWaiting())
            continue;

        int from = v.getCurrentFrom();
        int to = v.getCurrentTo();
        if (from < 0 || to < 0)
            continue;

        auto& edges = graph.GetAdjacencyMutable().at(from);

        for (auto& edge : edges)
        {
            if (edge.to != to || edge.isBlocked)
                continue;

            int bestLane = -1;
            int minLoad = INT_MAX;

            for (int i = 0; i < edge.lanes.size(); ++i)
            {
                if (edge.lanes[i].currentLoad <
                    edge.lanes[i].capacity &&
                    edge.lanes[i].currentLoad < minLoad)
                {
                    minLoad = edge.lanes[i].currentLoad;
                    bestLane = i;
                }
            }

            if (bestLane >= 0)
            {
                v.assignLane(bestLane);
                v.setWaiting(false);
                v.advanceToNextEdge(currentTime);
            }

            break;
        }
    }
}

// ======================================================
// Update Vehicles
// ======================================================

void TrafficEngine::UpdateVehicles(double deltaTime)
{
    // -------------------------------------------------
    // 1️⃣ Build lane grouping structure
    // -------------------------------------------------

    struct LaneKey
    {
        int from;
        int to;
        int lane;

        bool operator<(const LaneKey& other) const
        {
            if (from != other.from) return from < other.from;
            if (to != other.to) return to < other.to;
            return lane < other.lane;
        }
    };

    std::map<LaneKey, std::vector<Vehicle*>> laneMap;

    for (auto& v : vehicles)
    {
        if (v.hasFinished() || v.isWaiting())
            continue;

        int from = v.getCurrentFrom();
        int to = v.getCurrentTo();
        int lane = v.getCurrentLaneIndex();

        if (from < 0 || to < 0 || lane < 0)
            continue;

        laneMap[{from, to, lane}].push_back(&v);
    }

    // -------------------------------------------------
    // 2️⃣ Sort vehicles in each lane by position
    // -------------------------------------------------

    for (auto& pair : laneMap)
    {
        auto& laneVehicles = pair.second;

        std::sort(laneVehicles.begin(),
            laneVehicles.end(),
            [](Vehicle* a, Vehicle* b)
            {
                return a->getDistanceOnEdge() <
                    b->getDistanceOnEdge();
            });

        // -------------------------------------------------
        // 3️⃣ Apply IDM
        // -------------------------------------------------

        for (size_t i = 0; i < laneVehicles.size(); ++i)
        {
            Vehicle* current = laneVehicles[i];

            double desiredSpeed = 0.0;

            // Get edge
            int from = current->getCurrentFrom();
            int to = current->getCurrentTo();

            const auto& edges = graph.GetAdjacency().at(from);

            const Edge* edgePtr = nullptr;

            for (const auto& e : edges)
                if (e.to == to)
                {
                    edgePtr = &e;
                    break;
                }

            if (!edgePtr)
                continue;

            const Lane& lane =
                edgePtr->lanes[current->getCurrentLaneIndex()];

            desiredSpeed = lane.currentSpeed;

            // Leader?
            if (i == laneVehicles.size() - 1)
            {
                // No leader
                double accel =
                    current->computeIDMAcceleration(
                        current->getCurrentSpeed(),
                        desiredSpeed,
                        1e6,  // infinite gap
                        desiredSpeed);

                double newSpeed =
                    current->getCurrentSpeed() +
                    accel * deltaTime;

                current->setSpeed(std::max(0.0, newSpeed));
            }
            else
            {
                Vehicle* leader = laneVehicles[i + 1];

                double gap =
                    leader->getDistanceOnEdge() -
                    current->getDistanceOnEdge();

                gap = std::max(0.1, gap);

                double accel =
                    current->computeIDMAcceleration(
                        current->getCurrentSpeed(),
                        desiredSpeed,
                        gap,
                        leader->getCurrentSpeed());

                double newSpeed =
                    current->getCurrentSpeed() +
                    accel * deltaTime;

                current->setSpeed(std::max(0.0, newSpeed));
            }
        }
    }

    // -------------------------------------------------
    // 4️⃣ Move vehicles
    // -------------------------------------------------

  // -------------------------------------------------
// 4️⃣ Move vehicles + Edge Exit Detection
// -------------------------------------------------

    for (auto& v : vehicles)
    {
        if (v.hasFinished())
            continue;

        int oldFrom = v.getCurrentFrom();
        int oldTo = v.getCurrentTo();

        v.update(deltaTime, currentTime);

        int newFrom = v.getCurrentFrom();
        int newTo = v.getCurrentTo();

        // Detect edge transition (vehicle left an edge)
        if (oldFrom >= 0 && oldTo >= 0)
        {
            if (newFrom != oldFrom || newTo != oldTo)
            {
                EdgeKey key{ oldFrom, oldTo };

                auto it = edgeFDMap.find(key);
                if (it != edgeFDMap.end())
                {
                    it->second.exitCounter++;
                }
                else
                {
                    // First time seeing this edge
                    edgeFDMap[key].exitCounter = 1;
                }
            }
        }
    }
}

// ======================================================
// Remove Finished Vehicles
// ======================================================

void TrafficEngine::RemoveFinishedVehicles()
{
    vehicles.erase(
        std::remove_if(
            vehicles.begin(),
            vehicles.end(),
            [&](const Vehicle& v)
            {
                if (v.hasFinished())
                {
                    completedVehicles++;
                    totalTravelTime += v.getTravelTime();
                    return true;
                }
                return false;
            }),
        vehicles.end());
}

// ======================================================
// Congestion Detection
// ======================================================

void TrafficEngine::DetectCongestion(double deltaTime)
{
    double inflowRate = 1.0 / spawnInterval;
    double outflowRate = currentTime > 0
        ? completedVehicles / currentTime
        : 0.0;

    if (outflowRate < inflowRate * 0.85)
        growthTimer += deltaTime;
    else
        growthTimer = std::max(0.0, growthTimer - deltaTime);

    // Enter congestion
    if (!congestionDetected && growthTimer > 3.0)
        congestionDetected = true;

    // Exit congestion
    if (congestionDetected && growthTimer < 1.0)
        congestionDetected = false;
}

// ======================================================
// Snapshots
// ======================================================

std::vector<VehicleSnapshot> TrafficEngine::GetVehicleSnapshots() const
{
    std::vector<VehicleSnapshot> snapshots;

    for (const auto& v : vehicles)
    {
        snapshots.push_back({
            v.getId(),
            v.getX(),
            v.getY()
            });
    }

    return snapshots;
}

// ======================================================
// Metrics
// ======================================================

Metrics TrafficEngine::GetMetrics() const
{
    Metrics m;

    m.activeVehicles = static_cast<int>(vehicles.size());
    m.completedVehicles = completedVehicles;

    if (completedVehicles > 0)
        m.averageTravelTime = totalTravelTime / completedVehicles;

    if (currentTime > 0)
        m.throughput = completedVehicles / currentTime;

    m.congestionDetected = congestionDetected;

    return m;
}


// ======================================================
// Global FD Access
// ======================================================

GlobalFDData TrafficEngine::GetGlobalFDData() const
{
    return globalFD;
}


// ======================================================
// Edge FD Access
// ======================================================

bool TrafficEngine::GetEdgeFDData(
    int from,
    int to,
    EdgeFDData& outData) const
{
    EdgeKey key{ from, to };

    auto it = edgeFDMap.find(key);
    if (it == edgeFDMap.end())
        return false;

    outData = it->second;
    return true;
}

// ======================================================
// Fundamental Diagram Summary (Diagnostic)
// ======================================================

void TrafficEngine::PrintFundamentalDiagramSummary() const
{
    std::cout << "\n===== GLOBAL FUNDAMENTAL DIAGRAM =====\n";
    std::cout << "Max Observed Flow (Capacity): "
        << globalFD.maxObservedFlow << "\n";
    std::cout << "Critical Density: "
        << globalFD.criticalDensity << "\n";
    std::cout << "Total Samples: "
        << globalFD.samples << "\n";

    std::cout << "\n===== PER-EDGE FUNDAMENTAL DIAGRAM =====\n";

    for (const auto& pair : edgeFDMap)
    {
        const EdgeKey& key = pair.first;
        const EdgeFDData& data = pair.second;

        std::cout << "Edge (" << key.from
            << " -> " << key.to << ")\n";

        std::cout << "  Max Flow: "
            << data.maxObservedFlow << "\n";

        std::cout << "  Critical Density: "
            << data.criticalDensity << "\n";

        std::cout << "  Samples: "
            << data.samples << "\n";
    }

    std::cout << "=======================================\n";
}


// ======================================================
// Fundamental Diagram Processing
// ======================================================

void TrafficEngine::UpdateFundamentalDiagram(double deltaTime)
{
    fdTimer += deltaTime;

    if (fdTimer < fdSamplingInterval)
        return;

    // ==========================================
    // 1️⃣ Collect instantaneous per-edge data
    // ==========================================

    struct TempData
    {
        int vehicleCount = 0;
        double speedSum = 0.0;
    };

    std::unordered_map<EdgeKey, TempData, EdgeKeyHash> tempMap;

    // Count vehicles and speeds per edge
    for (const auto& v : vehicles)
    {
        if (v.hasFinished() || v.isWaiting())
            continue;

        int from = v.getCurrentFrom();
        int to = v.getCurrentTo();

        if (from < 0 || to < 0)
            continue;

        EdgeKey key{ from, to };

        tempMap[key].vehicleCount++;
        tempMap[key].speedSum += v.getCurrentSpeed();
    }

    double totalVehicles = 0.0;
    double totalLength = 0.0;
    double totalSpeedSum = 0.0;
    int totalExits = 0;

    // ==========================================
    // 2️⃣ Compute FD metrics per edge
    // ==========================================

    for (auto& pair : graph.GetAdjacency())
    {
        int from = pair.first;

        for (const auto& edge : pair.second)
        {
            EdgeKey key{ from, edge.to };

            int vehicleCount = 0;
            double speedSum = 0.0;

            auto tempIt = tempMap.find(key);
            if (tempIt != tempMap.end())
            {
                vehicleCount = tempIt->second.vehicleCount;
                speedSum = tempIt->second.speedSum;
            }

            double density = 0.0;
            double avgSpeed = 0.0;

            if (edge.length > 0.0)
                density = vehicleCount / edge.length;

            if (vehicleCount > 0)
                avgSpeed = speedSum / vehicleCount;

            double flow = 0.0;

            auto fdIt = edgeFDMap.find(key);
            if (fdIt != edgeFDMap.end())
            {
                flow = fdIt->second.exitCounter / fdSamplingInterval;
            }

            // Store instantaneous
            EdgeFDData& fdData = edgeFDMap[key];

            fdData.density = density;
            fdData.flow = flow;
            fdData.averageSpeed = avgSpeed;

            fdData.densitySum += density;
            fdData.flowSum += flow;
            fdData.speedSum += avgSpeed;
            fdData.samples++;


            // ==========================================
            // Curve History Tracking
            // ==========================================

            fdData.densityHistory.push_back(density);
            fdData.flowHistory.push_back(flow);
            fdData.speedHistory.push_back(avgSpeed);

            // Capacity tracking
            if (flow > fdData.maxObservedFlow)
            {
                fdData.maxObservedFlow = flow;
                fdData.criticalDensity = density;
            }



            // Global accumulation
            totalVehicles += vehicleCount;
            totalLength += edge.length;
            totalSpeedSum += speedSum;
            totalExits += fdData.exitCounter;

            // Reset exit counter
            fdData.exitCounter = 0;
        }
    }

    // ==========================================
    // 3️⃣ Compute global FD metrics
    // ==========================================

    if (totalLength > 0.0)
        globalFD.density = totalVehicles / totalLength;

    if (fdSamplingInterval > 0.0)
        globalFD.flow = totalExits / fdSamplingInterval;

    if (totalVehicles > 0)
        globalFD.averageSpeed = totalSpeedSum / totalVehicles;

    globalFD.densitySum += globalFD.density;
    globalFD.flowSum += globalFD.flow;
    globalFD.speedSum += globalFD.averageSpeed;
    globalFD.samples++;


    // ==========================================
    // Global Curve History Tracking
    // ==========================================

    globalFD.densityHistory.push_back(globalFD.density);
    globalFD.flowHistory.push_back(globalFD.flow);
    globalFD.speedHistory.push_back(globalFD.averageSpeed);

    // Global capacity tracking
    if (globalFD.flow > globalFD.maxObservedFlow)
    {
        globalFD.maxObservedFlow = globalFD.flow;
        globalFD.criticalDensity = globalFD.density;
    }


    // Reset sampling timer
    fdTimer = 0.0;
}