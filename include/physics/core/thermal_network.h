#pragma once

#include <cstddef>
#include <set>
#include <utility>
#include <vector>

namespace PhysicsEngine {

struct ThermalNode {
    double temperature; // Kelvin, nonnegative.
    double heatCapacity; // J/K, strictly positive.
    bool fixed;
    double externalPower = 0.0; // W, queued until a successful positive-dt step.
    double reservoirHeat = 0.0; // J supplied by this thermostat, cumulative.
};

struct ThermalLink {
    std::size_t first;
    std::size_t second;
    double conductance; // W/K, nonnegative.
};

struct ThermalNetworkConfig {
    double maxSubstep = 0.01;
    double safetyFactor = 0.9; // (0,1], h*sum(G)/C <= safetyFactor.
    std::size_t maxSubsteps = 4096;
    std::size_t maxNodes = 100000;
    std::size_t maxLinks = 300000;
};

struct ThermalDiagnostics {
    double totalEnergy = 0.0; // sum(C*T), includes constant fixed-node energies.
    double minimumTemperature = 0.0;
    double maximumTemperature = 0.0;
    double totalExternalEnergy = 0.0; // J, accepted steps, includes reservoir loads.
    double totalReservoirHeat = 0.0; // J, positive when thermostats supply the graph.
    double lastExternalEnergy = 0.0;
    double lastReservoirHeat = 0.0;
    std::size_t lastSubsteps = 0;
};

// Standalone lumped heat-capacity graph, explicit Euler. Append-only indexes.
// No automatic mechanical coupling, radiation, phase changes or fluid advection.
class ThermalNetwork {
public:
    explicit ThermalNetwork(const ThermalNetworkConfig& config = {});
    std::size_t addNode(double temperature, double heatCapacity, bool fixed = false);
    std::size_t addLink(std::size_t first, std::size_t second, double conductance);
    void setTemperature(std::size_t index, double temperature);
    void setFixed(std::size_t index, bool fixed);
    void applyPower(std::size_t index, double power);
    void clearPowers() noexcept;
    void clearPowers(std::size_t index);
    void setConfig(const ThermalNetworkConfig& config);
    const ThermalNetworkConfig& getConfig() const noexcept { return config_; }
    const std::vector<ThermalNode>& getNodes() const noexcept { return nodes_; }
    const std::vector<ThermalLink>& getLinks() const noexcept { return links_; }
    ThermalDiagnostics getDiagnostics() const;
    // Failures preserve temperatures, loads and all accounting. Zero dt keeps
    // queued powers; a successful positive dt consumes them, including fixed nodes.
    void step(double dt);

private:
    ThermalNetworkConfig config_;
    std::vector<ThermalNode> nodes_;
    std::vector<ThermalLink> links_;
    std::set<std::pair<std::size_t, std::size_t>> linkPairs_;
    double totalExternalEnergy_ = 0.0;
    double totalReservoirHeat_ = 0.0;
    double lastExternalEnergy_ = 0.0;
    double lastReservoirHeat_ = 0.0;
    std::size_t lastSubsteps_ = 0;
};

} // namespace PhysicsEngine
