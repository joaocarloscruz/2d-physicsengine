#include "physics/core/thermal_network.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {
namespace {
void validateConfig(const ThermalNetworkConfig& c) {
    if (!std::isfinite(c.maxSubstep) || c.maxSubstep <= 0.0 ||
        !std::isfinite(c.safetyFactor) || c.safetyFactor <= 0.0 || c.safetyFactor > 1.0 ||
        c.maxSubsteps == 0 || c.maxNodes == 0 || c.maxLinks == 0)
        throw std::invalid_argument("Invalid thermal network configuration");
}
void validateTemperature(double temperature, double capacity) {
    if (!std::isfinite(temperature) || temperature < 0.0 || !std::isfinite(capacity * temperature))
        throw std::invalid_argument("Invalid thermal temperature or node energy");
}
double checked(double value) {
    if (!std::isfinite(value)) throw std::runtime_error("Thermal network arithmetic overflow");
    return value;
}
double totalEnergy(const std::vector<ThermalNode>& nodes) {
    double result = 0.0;
    for (const auto& node : nodes) result = checked(result + checked(node.heatCapacity * node.temperature));
    return result;
}
double maximumSubstep(const std::vector<ThermalNode>& nodes,
                      const std::vector<ThermalLink>& links, const ThermalNetworkConfig& c) {
    std::vector<double> conductance(nodes.size(), 0.0);
    for (const auto& link : links) {
        if (!nodes[link.first].fixed)
            conductance[link.first] = checked(conductance[link.first] + link.conductance);
        if (!nodes[link.second].fixed)
            conductance[link.second] = checked(conductance[link.second] + link.conductance);
    }
    double h = c.maxSubstep;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (conductance[i] == 0.0) continue;
        const double rate = checked(conductance[i] / nodes[i].heatCapacity);
        if (rate > 0.0) h = std::min(h, c.safetyFactor / rate);
    }
    return h;
}
} // namespace

ThermalNetwork::ThermalNetwork(const ThermalNetworkConfig& config) : config_(config) { validateConfig(config); }

std::size_t ThermalNetwork::addNode(double temperature, double heatCapacity, bool fixed) {
    if (!std::isfinite(heatCapacity) || heatCapacity <= 0.0 || !std::isfinite(1.0 / heatCapacity))
        throw std::invalid_argument("Invalid thermal heat capacity");
    validateTemperature(temperature, heatCapacity);
    if (nodes_.size() >= config_.maxNodes) throw std::length_error("Thermal node budget exceeded");
    nodes_.push_back({temperature, heatCapacity, fixed, 0.0, 0.0});
    return nodes_.size() - 1;
}
std::size_t ThermalNetwork::addLink(std::size_t first, std::size_t second, double conductance) {
    if (first >= nodes_.size() || second >= nodes_.size()) throw std::out_of_range("Thermal link node index");
    if (first == second || !std::isfinite(conductance) || conductance < 0.0)
        throw std::invalid_argument("Invalid thermal conductance link");
    const auto pair = std::minmax(first, second);
    if (linkPairs_.count(pair)) throw std::invalid_argument("Duplicate thermal link");
    if (links_.size() >= config_.maxLinks) throw std::length_error("Thermal link budget exceeded");
    const auto inserted = linkPairs_.insert(pair);
    try { links_.push_back({first, second, conductance}); }
    catch (...) { linkPairs_.erase(inserted.first); throw; }
    return links_.size() - 1;
}
void ThermalNetwork::setTemperature(std::size_t index, double temperature) {
    auto& node = nodes_.at(index);
    validateTemperature(temperature, node.heatCapacity);
    node.temperature = temperature;
}
void ThermalNetwork::setFixed(std::size_t index, bool fixed) { nodes_.at(index).fixed = fixed; }
void ThermalNetwork::applyPower(std::size_t index, double power) {
    auto& node = nodes_.at(index);
    if (!std::isfinite(power)) throw std::invalid_argument("Invalid thermal external power");
    node.externalPower = checked(node.externalPower + power);
}
void ThermalNetwork::clearPowers() noexcept {
    for (auto& node : nodes_) node.externalPower = 0.0;
}
void ThermalNetwork::clearPowers(std::size_t index) { nodes_.at(index).externalPower = 0.0; }
void ThermalNetwork::setConfig(const ThermalNetworkConfig& config) {
    validateConfig(config);
    if (nodes_.size() > config.maxNodes || links_.size() > config.maxLinks)
        throw std::length_error("Thermal configuration excludes existing topology");
    config_ = config;
}

void ThermalNetwork::step(double dt) {
    if (!std::isfinite(dt) || dt < 0.0) throw std::invalid_argument("Invalid thermal timestep");
    if (dt == 0.0 || nodes_.empty()) {
        lastSubsteps_ = 0;
        lastExternalEnergy_ = lastReservoirHeat_ = 0.0;
        return;
    }
    const double limit = maximumSubstep(nodes_, links_, config_);
    const double requested = dt / limit;
    const double roundoff = 64.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, requested);
    if (!(limit > 0.0) || !std::isfinite(requested) ||
        requested > static_cast<double>(config_.maxSubsteps) + roundoff)
        throw std::runtime_error("Thermal substep budget exceeded");
    const double count = std::max(1.0, std::ceil(requested - roundoff));
    // Avoid converting a rounded SIZE_MAX double to an out-of-range size_t.
    const std::size_t substeps = count >= static_cast<double>(config_.maxSubsteps)
        ? config_.maxSubsteps : static_cast<std::size_t>(count);
    const double h = dt / static_cast<double>(substeps);
    if (!(h > 0.0)) throw std::runtime_error("Thermal timestep underflow");
    auto state = nodes_;
    std::vector<double> heat(state.size());
    double externalEnergy = 0.0;
    double reservoirHeat = 0.0;
    for (std::size_t stepIndex = 0; stepIndex < substeps; ++stepIndex) {
        for (std::size_t i = 0; i < state.size(); ++i) {
            heat[i] = checked(state[i].externalPower * h);
            externalEnergy = checked(externalEnergy + heat[i]);
        }
        // Equal/opposite transfers use the same old temperatures: explicit Euler.
        for (const auto& link : links_) {
            const double transfer = checked(h * checked(link.conductance *
                (state[link.second].temperature - state[link.first].temperature)));
            heat[link.first] = checked(heat[link.first] + transfer);
            heat[link.second] = checked(heat[link.second] - transfer);
        }
        for (std::size_t i = 0; i < state.size(); ++i) {
            if (state[i].fixed) {
                reservoirHeat = checked(reservoirHeat - heat[i]);
                state[i].reservoirHeat = checked(state[i].reservoirHeat - heat[i]);
            } else {
                const double temperature = checked(state[i].temperature + checked(heat[i] / state[i].heatCapacity));
                if (temperature < 0.0) throw std::runtime_error("Thermal external cooling crossed zero Kelvin");
                checked(state[i].heatCapacity * temperature);
                state[i].temperature = temperature;
            }
        }
    }
    // All accounting and diagnostic representability must pass before committing.
    totalEnergy(state);
    const double nextExternal = checked(totalExternalEnergy_ + externalEnergy);
    const double nextReservoir = checked(totalReservoirHeat_ + reservoirHeat);
    for (auto& node : state) node.externalPower = 0.0;
    nodes_.swap(state);
    totalExternalEnergy_ = nextExternal;
    totalReservoirHeat_ = nextReservoir;
    lastExternalEnergy_ = externalEnergy;
    lastReservoirHeat_ = reservoirHeat;
    lastSubsteps_ = substeps;
}

ThermalDiagnostics ThermalNetwork::getDiagnostics() const {
    ThermalDiagnostics result;
    result.totalEnergy = totalEnergy(nodes_);
    if (!nodes_.empty()) {
        result.minimumTemperature = result.maximumTemperature = nodes_.front().temperature;
        for (const auto& node : nodes_) {
            result.minimumTemperature = std::min(result.minimumTemperature, node.temperature);
            result.maximumTemperature = std::max(result.maximumTemperature, node.temperature);
        }
    }
    result.totalExternalEnergy = totalExternalEnergy_;
    result.totalReservoirHeat = totalReservoirHeat_;
    result.lastExternalEnergy = lastExternalEnergy_;
    result.lastReservoirHeat = lastReservoirHeat_;
    result.lastSubsteps = lastSubsteps_;
    return result;
}
} // namespace PhysicsEngine
