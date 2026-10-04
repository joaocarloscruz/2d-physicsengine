#include "physics/core/thermal_network.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <initializer_list>
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
// Multiply the complete transfer without overflow/underflow in an intermediate
// product. frexp exponents sum to at most about 3*1074 for finite doubles.
double heatTransfer(double h, double conductance, double difference) {
    if (conductance == 0.0 || difference == 0.0) return 0.0;
    int eh, eg, et;
    const double mh = std::frexp(h, &eh);
    const double mg = std::frexp(conductance, &eg);
    const double mt = std::frexp(difference, &et);
    return checked(std::scalbn((mh * mg) * mt, eh + eg + et));
}
// Keep complete products representable even when T^4 or an intermediate
// power/rate is outside the double range. At most eight factors are used.
double scaledProduct(std::initializer_list<double> factors, double divisor = 1.0) {
    double mantissa = 1.0;
    int exponent = 0;
    for (double value : factors) {
        if (value == 0) return 0;
        int e;
        mantissa *= std::frexp(value, &e);
        exponent += e;
    }
    int e;
    mantissa /= std::frexp(divisor, &e);
    return checked(std::scalbn(mantissa, exponent - e));
}
double radiationTransfer(double h, double coefficient, double a, double b) {
    if (a == b || coefficient == 0) return 0;
    const double high = std::max(a, b), ratio = std::min(a, b) / high;
    // (b^4-a^4) = (b-a)*high^3*(1+ratio)*(1+ratio^2).
    return scaledProduct({h, coefficient, b-a, high, high, high,
                          1+ratio, 1+ratio*ratio});
}

double radiativeSubstep(const std::vector<ThermalNode>& nodes,
                       const std::vector<ThermalLink>& links,
                       const std::vector<ThermalRadiationLink>& radiation,
                       const ThermalNetworkConfig& config) {
    std::vector<double> rates(nodes.size(), 0);
    std::vector<std::size_t> degree(nodes.size(), 0);
    const double infinity = std::numeric_limits<double>::infinity();
    auto add = [&](std::size_t index, double rate) {
        // Margin includes exponent-product rounding and a possibly underflowed
        // positive rate. Summation is directed outward as well.
        rate = checked(std::nextafter(checked(rate * (1+16*std::numeric_limits<double>::epsilon())), infinity));
        rates[index] = checked(std::nextafter(checked(rates[index] + rate), infinity));
        ++degree[index];
    };
    for (const auto& link : links) {
        if (link.conductance == 0) continue;
        for (const auto i : {link.first, link.second})
            if (!nodes[i].fixed) add(i, checked(link.conductance / nodes[i].heatCapacity));
    }
    for (const auto& link : radiation) {
        const double high = std::max(nodes[link.first].temperature, nodes[link.second].temperature);
        if (link.coefficient == 0 || high == 0) continue;
        // The nonnegative secant conductance is <= 4*kappa*max(Ta,Tb)^3.
        // Recomputed from the current staged temperatures before every Euler
        // substep, so queued heating cannot invalidate a frozen rate bound.
        for (const auto i : {link.first, link.second})
            if (!nodes[i].fixed)
                add(i, scaledProduct({4, link.coefficient, high, high, high}, nodes[i].heatCapacity));
    }
    double limit = config.maxSubstep;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (rates[i] == 0) continue;
        const double allowance = 16*std::numeric_limits<double>::epsilon()*(double(degree[i])+1);
        if (allowance >= .5) throw std::runtime_error("Thermal graph exceeds bound precision");
        const double bound = std::nextafter((config.safetyFactor/rates[i])*(1-allowance), 0.0);
        limit = std::min(limit, bound);
    }
    if (!(limit > 0)) throw std::runtime_error("Thermal radiative timestep underflow");
    return limit;
}
double totalEnergy(const std::vector<ThermalNode>& nodes) {
    double result = 0.0;
    for (const auto& node : nodes) result = checked(result + checked(node.heatCapacity * node.temperature));
    return result;
}
double conductionSubstep(const std::vector<ThermalNode>& nodes,
                      const std::vector<ThermalLink>& links, const ThermalNetworkConfig& c) {
    std::vector<double> conductance(nodes.size(), 0.0);
    std::vector<std::size_t> degree(nodes.size(), 0);
    for (const auto& link : links) {
        if (link.conductance == 0.0) continue;
        if (!nodes[link.first].fixed) {
            conductance[link.first] = checked(std::nextafter(
                checked(conductance[link.first] + link.conductance), std::numeric_limits<double>::infinity()));
            ++degree[link.first];
        }
        if (!nodes[link.second].fixed) {
            conductance[link.second] = checked(std::nextafter(
                checked(conductance[link.second] + link.conductance), std::numeric_limits<double>::infinity()));
            ++degree[link.second];
        }
    }
    double h = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (conductance[i] == 0.0) continue;
        const double rate = checked(std::nextafter(checked(conductance[i] / nodes[i].heatCapacity),
                                                  std::numeric_limits<double>::infinity()));
        // Directed rounding bounds the true row sum/rate; a small additional
        // margin covers transfer accumulation and the final temperature divide.
        const double allowance = 16.0 * std::numeric_limits<double>::epsilon() *
                                 (static_cast<double>(degree[i]) + 1.0);
        if (allowance >= 0.5) throw std::runtime_error("Thermal graph exceeds bound precision");
        const double margin = 1.0 - allowance;
        const double bound = std::nextafter((c.safetyFactor / rate) * margin, 0.0);
        h = std::min(h, bound);
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
    if (links_.size() >= config_.maxLinks - radiationLinks_.size()) throw std::length_error("Thermal link budget exceeded");
    const auto inserted = linkPairs_.insert(pair);
    try { links_.push_back({first, second, conductance}); }
    catch (...) { linkPairs_.erase(inserted.first); throw; }
    return links_.size() - 1;
}
std::size_t ThermalNetwork::addRadiationLink(std::size_t first, std::size_t second, double coefficient) {
    if (first >= nodes_.size() || second >= nodes_.size()) throw std::out_of_range("Thermal radiation node index");
    if (first == second || !std::isfinite(coefficient) || coefficient < 0)
        throw std::invalid_argument("Invalid thermal radiation link");
    const auto pair = std::minmax(first, second);
    if (radiationPairs_.count(pair)) throw std::invalid_argument("Duplicate thermal radiation link");
    if (radiationLinks_.size() >= config_.maxLinks - links_.size())
        throw std::length_error("Thermal link budget exceeded");
    const auto inserted = radiationPairs_.insert(pair);
    try { radiationLinks_.push_back({first, second, coefficient}); }
    catch (...) { radiationPairs_.erase(inserted.first); throw; }
    return radiationLinks_.size()-1;
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
    if (nodes_.size() > config.maxNodes || links_.size() > config.maxLinks ||
        radiationLinks_.size() > config.maxLinks - links_.size())
        throw std::length_error("Thermal configuration excludes existing topology");
    config_ = config;
}

void ThermalNetwork::step(double dt) {
    if (!std::isfinite(dt) || dt < 0.0) throw std::invalid_argument("Invalid thermal timestep");
    if (dt == 0.0 || nodes_.empty()) {
        lastSubsteps_ = 0;
        lastRadiativeVisits_ = 0;
        lastExternalEnergy_ = lastReservoirHeat_ = 0.0;
        return;
    }
    if (!radiationLinks_.empty()) {
        stepRadiative(dt);
        return;
    }
    const double conductionLimit = conductionSubstep(nodes_, links_, config_);
    const double limit = std::min(config_.maxSubstep, conductionLimit);
    const double requested = dt / limit;
    const double roundoff = 64.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, requested);
    if (!(limit > 0.0) || !std::isfinite(requested) ||
        requested > static_cast<double>(config_.maxSubsteps) + roundoff)
        throw std::runtime_error("Thermal substep budget exceeded");
    double count = std::max(1.0, std::ceil(requested - roundoff));
    // Decimal maxSubstep partitions can use roundoff tolerance, but never let
    // that tolerance enlarge a physical conduction bound at safetyFactor=1.
    if (dt / count > conductionLimit) {
        count = std::ceil(dt / conductionLimit);
        if (dt / count > conductionLimit) {
            if (count + 1.0 == count) throw std::runtime_error("Thermal substep count precision exhausted");
            count += 1.0;
        }
    }
    if (count > static_cast<double>(config_.maxSubsteps))
        throw std::runtime_error("Thermal substep budget exceeded");
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
            const double transfer = heatTransfer(h, link.conductance,
                state[link.second].temperature - state[link.first].temperature);
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
    lastRadiativeVisits_ = 0;
}

void ThermalNetwork::stepRadiative(double dt) {
    // Six charged node sweeps cover scratch initialization, rate reduction,
    // queued loads and updates; both kinds of edge are visited for bounds and
    // transfers. Copies and final diagnostics are bounded by this same topology.
    constexpr auto budget = ThermalNetworkConfig::MaximumRadiativeVisits;
    if (nodes_.size() > budget/6 || links_.size() > budget/2 || radiationLinks_.size() > budget/2)
        throw std::runtime_error("Thermal radiative work budget exceeded");
    const std::size_t visits = 6*nodes_.size() + 2*links_.size() + 2*radiationLinks_.size();
    if (visits > budget) throw std::runtime_error("Thermal radiative work budget exceeded");
    auto state = nodes_;
    std::vector<double> heat(state.size());
    double elapsed = 0, externalEnergy = 0, reservoirHeat = 0;
    std::size_t substeps = 0, work = 0;
    while (elapsed < dt) {
        if (substeps >= config_.maxSubsteps) throw std::runtime_error("Thermal substep budget exceeded");
        if (visits > budget-work) throw std::runtime_error("Thermal radiative work budget exceeded");
        work += visits;
        const double limit = radiativeSubstep(state, links_, radiationLinks_, config_);
        const double desired = std::min(dt-elapsed, limit);
        double next = elapsed + desired;
        if (next-elapsed > limit) next = std::nextafter(next, elapsed);
        if (!(next > elapsed) || next > dt || !std::isfinite(next))
            throw std::runtime_error("Thermal radiative time resolution exhausted");
        const double h = next-elapsed;
        for (std::size_t i = 0; i < state.size(); ++i) {
            heat[i] = checked(state[i].externalPower*h);
            externalEnergy = checked(externalEnergy+heat[i]);
        }
        auto transfer = [&](std::size_t a, std::size_t b, double energy) {
            heat[a] = checked(heat[a]+energy);
            heat[b] = checked(heat[b]-energy);
        };
        for (const auto& link : links_)
            transfer(link.first, link.second, heatTransfer(h, link.conductance,
                state[link.second].temperature-state[link.first].temperature));
        for (const auto& link : radiationLinks_)
            transfer(link.first, link.second, radiationTransfer(h, link.coefficient,
                state[link.first].temperature, state[link.second].temperature));
        for (std::size_t i = 0; i < state.size(); ++i) {
            if (state[i].fixed) {
                reservoirHeat = checked(reservoirHeat-heat[i]);
                state[i].reservoirHeat = checked(state[i].reservoirHeat-heat[i]);
            } else {
                const double temperature = checked(state[i].temperature+checked(heat[i]/state[i].heatCapacity));
                if (temperature < 0) throw std::runtime_error("Thermal external cooling crossed zero Kelvin");
                checked(state[i].heatCapacity*temperature);
                state[i].temperature = temperature;
            }
        }
        elapsed = next;
        ++substeps;
    }
    totalEnergy(state);
    const double nextExternal = checked(totalExternalEnergy_+externalEnergy);
    const double nextReservoir = checked(totalReservoirHeat_+reservoirHeat);
    for (auto& node : state) node.externalPower = 0;
    nodes_.swap(state);
    totalExternalEnergy_ = nextExternal;
    totalReservoirHeat_ = nextReservoir;
    lastExternalEnergy_ = externalEnergy;
    lastReservoirHeat_ = reservoirHeat;
    lastSubsteps_ = substeps;
    lastRadiativeVisits_ = work;
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
    result.lastRadiativeVisits = lastRadiativeVisits_;
    return result;
}
} // namespace PhysicsEngine
