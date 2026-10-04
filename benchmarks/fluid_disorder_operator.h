#pragma once
// Diagnostic-only, independently derived double operators. Never used by a
// production solver. The fixed scope is homogeneous h/rho0 and <=2048 points.
#include "physics/core/fluids/wcsph_solver.h"
#include <cmath>
#include <stdexcept>
#include <vector>

namespace FluidDisorder {
constexpr double Pi = 3.141592653589793238462643383279502884;
constexpr float OriginalMassScale = 1.0f / 1.014612675f;
struct D2 {
    double x = 0, y = 0;
    D2 operator+(D2 b) const { return {x + b.x, y + b.y}; }
    D2 operator-(D2 b) const { return {x - b.x, y - b.y}; }
    D2 operator*(double s) const { return {x * s, y * s}; }
    double dot(D2 b) const { return x * b.x + y * b.y; }
    double norm() const { return std::hypot(x, y); }
};
using Family = PhysicsEngine::SphKernelFamily;
inline double Weight(D2 r, double h, Family family) {
    const double q = r.norm() / h;
    if (q >= 1)
        return 0;
    if (family == Family::Poly6Spiky)
        return 4 / (Pi * h * h) * std::pow(1 - q * q, 3);
    const double f = q < .5 ? 1 - 6 * q * q + 6 * q * q * q : 2 * std::pow(1 - q, 3);
    return 40 / (7 * Pi * h * h) * f;
}
inline D2 Gradient(D2 r, double h, Family family, bool densityDerivative) {
    const double length = r.norm(), q = length / h;
    if (length == 0 || q >= 1)
        return {};
    double derivative;
    if (family == Family::CubicSpline)
        derivative =
            40 / (7 * Pi * h * h * h) * (q < .5 ? -12 * q + 18 * q * q : -6 * (1 - q) * (1 - q));
    else if (densityDerivative)
        derivative = -24 / (Pi * h * h * h) * q * std::pow(1 - q * q, 2);
    else
        derivative = -30 / (Pi * h * h * h) * std::pow(1 - q, 2);
    return r * (derivative / length);
}
inline double Pressure(double ratio, bool clamp, double c = 15, double rho0 = 1000,
                       double gamma = 7) {
    const double p = rho0 * c * c / gamma * (std::pow(ratio, gamma) - 1);
    return clamp && p < 0 ? 0 : p;
}
// Integral of du/drho=p/rho^2, u(rho0)=0. Clamped EOS is flat below rho0.
inline double SpecificEnergy(double ratio, bool clamp, double c = 15, double gamma = 7) {
    if (clamp && ratio <= 1)
        return 0;
    return c * c / gamma * ((std::pow(ratio, gamma - 1) - 1) / (gamma - 1) + 1 / ratio - 1);
}
inline std::vector<PhysicsEngine::FluidParticle> Block(int side, float dx, float h,
                                                       float massScale = OriginalMassScale,
                                                       float amplitude = .02f,
                                                       bool disturb = true) {
    if (side < 3 || side > 44)
        throw std::length_error("Disorder block exceeds diagnostic limits");
    PhysicsEngine::FluidParticleProperties props;
    props.mass = props.restDensity * dx * dx * massScale;
    props.smoothingLength = h;
    props.viscosity = .05f;
    std::vector<PhysicsEngine::FluidParticle> p;
    for (int y = 0; y < side; ++y)
        for (int x = 0; x < side; ++x) {
            PhysicsEngine::Vector2 position{x * dx, y * dx};
            if (disturb) {
                position.x += ((x + y) % 2 == 0 ? 1.f : -1.f) * dx * amplitude;
                position.y += ((3 * y + x) % 2 == 0 ? 1.f : -1.f) * dx * amplitude;
            }
            p.emplace_back(position, PhysicsEngine::Vector2{}, props);
        }
    return p;
}
struct BulkRow {
    double unshifted = 0, shifted = 0;
    D2 linearSymbol, gradientSum;
};
inline BulkRow LatticeRow(double dx, double h, double amplitude, Family family,
                          bool densityDerivative = true) {
    if (!std::isfinite(dx) || !std::isfinite(h) || !std::isfinite(amplitude) ||
        !(dx > 0 && h > 0) || h / dx > 16 || std::abs(amplitude) > .2)
        throw std::invalid_argument("Disorder lattice row exceeds diagnostic bounds");
    BulkRow row;
    const int reach = int(std::ceil(h / dx)) + 1;
    for (int y = -reach; y <= reach; ++y)
        for (int x = -reach; x <= reach; ++x) {
            const bool odd = (x + y) % 2 != 0;
            const D2 r{x * dx, y * dx},
                shift = odd ? D2{2 * amplitude * dx, 2 * amplitude * dx} : D2{};
            row.unshifted += dx * dx * Weight(r, h, family);
            row.shifted += dx * dx * Weight(r + shift, h, family);
            row.linearSymbol = row.linearSymbol +
                               Gradient(r, h, family, densityDerivative) * (odd ? 2 * dx * dx : 0);
            row.gradientSum =
                row.gradientSum + Gradient(r + shift, h, family, densityDerivative) * (dx * dx);
        }
    return row;
}
struct System {
    std::vector<D2> positions, velocities;
    std::vector<double> masses;
    Family family = Family::Poly6Spiky;
    double h = .2, rho0 = 1000, c = 15;
    bool clamp = true;
};
inline System FromParticles(const std::vector<PhysicsEngine::FluidParticle> &p, Family family) {
    if (p.empty() || p.size() > 2048)
        throw std::length_error("Disorder operator point budget exceeded");
    System s;
    s.family = family;
    s.h = p[0].smoothingLength;
    s.rho0 = p[0].restDensity;
    for (const auto &v : p) {
        if (v.smoothingLength != s.h || v.restDensity != s.rho0)
            throw std::invalid_argument("Disorder operator requires homogeneous h/rho0");
        s.positions.push_back({v.position.x, v.position.y});
        s.velocities.push_back({v.velocity.x, v.velocity.y});
        s.masses.push_back(v.mass);
    }
    return s;
}
struct Operators {
    std::vector<double> densities, pressures, densityRates;
    std::vector<D2> forces;
    double internalEnergy = 0, mechanicalWork = 0, trueEnergyRate = 0, pressureMapEnergyRate = 0;
};
inline Operators Build(const System &s) {
    const auto n = s.positions.size();
    if (n == 0 || n > 2048 || s.velocities.size() != n || s.masses.size() != n)
        throw std::length_error("Disorder operator shape or work budget exceeded");
    if (!std::isfinite(s.h) || s.h <= 0 || !std::isfinite(s.rho0) || s.rho0 <= 0 ||
        !std::isfinite(s.c) || s.c <= 0 ||
        (s.family != Family::Poly6Spiky && s.family != Family::CubicSpline))
        throw std::invalid_argument("Invalid disorder medium or family");
    for (std::size_t i = 0; i < n; ++i)
        if (!std::isfinite(s.positions[i].x) || !std::isfinite(s.positions[i].y) ||
            !std::isfinite(s.velocities[i].x) || !std::isfinite(s.velocities[i].y) ||
            !std::isfinite(s.masses[i]) || s.masses[i] <= 0)
            throw std::invalid_argument("Invalid disorder operator state");
    Operators o;
    o.densities.resize(n);
    o.pressures.resize(n);
    o.densityRates.resize(n);
    o.forces.resize(n);
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < n; ++j)
            o.densities[i] += s.masses[j] * Weight(s.positions[i] - s.positions[j], s.h, s.family);
    for (std::size_t i = 0; i < n; ++i) {
        if (!(std::isfinite(o.densities[i]) && o.densities[i] > 0))
            throw std::overflow_error("Invalid diagnostic density");
        o.pressures[i] = Pressure(o.densities[i] / s.rho0, s.clamp, s.c, s.rho0);
        o.internalEnergy += s.masses[i] * SpecificEnergy(o.densities[i] / s.rho0, s.clamp, s.c);
    }
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = i + 1; j < n; ++j) {
            const D2 r = s.positions[i] - s.positions[j], v = s.velocities[i] - s.velocities[j];
            const double coeff = s.masses[i] * s.masses[j] *
                                 (o.pressures[i] / (o.densities[i] * o.densities[i]) +
                                  o.pressures[j] / (o.densities[j] * o.densities[j]));
            const D2 force = Gradient(r, s.h, s.family, false) * (-coeff);
            o.forces[i] = o.forces[i] + force;
            o.forces[j] = o.forces[j] - force;
            const double densityRate = v.dot(Gradient(r, s.h, s.family, true));
            o.densityRates[i] += s.masses[j] * densityRate;
            o.densityRates[j] += s.masses[i] * densityRate;
            o.pressureMapEnergyRate += coeff * v.dot(Gradient(r, s.h, s.family, false));
        }
    for (std::size_t i = 0; i < n; ++i) {
        o.mechanicalWork += s.velocities[i].dot(o.forces[i]);
        o.trueEnergyRate +=
            s.masses[i] * o.pressures[i] / (o.densities[i] * o.densities[i]) * o.densityRates[i];
    }
    if (!std::isfinite(o.internalEnergy + o.mechanicalWork + o.trueEnergyRate +
                       o.pressureMapEnergyRate))
        throw std::overflow_error("Disorder diagnostic energy/work range exceeded");
    return o;
}
inline double EnergyDifference(const System &s, double dt) {
    if (!std::isfinite(dt) || dt <= 0)
        throw std::invalid_argument("Invalid difference step");
    auto plus = s, minus = s;
    for (std::size_t i = 0; i < s.positions.size(); ++i) {
        plus.positions[i] = s.positions[i] + s.velocities[i] * dt;
        minus.positions[i] = s.positions[i] - s.velocities[i] * dt;
    }
    return (Build(plus).internalEnergy - Build(minus).internalEnergy) / (2 * dt);
}
} // namespace FluidDisorder
