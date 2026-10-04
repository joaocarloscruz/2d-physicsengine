#pragma once
// Shared implementation arithmetic with internal linkage; not an installed API.
#include "physics/core/fluids/periodic_euler_gas_grid.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <utility>

namespace PhysicsEngine {
namespace {
double Checked(double x) {
    if (!std::isfinite(x))
        throw std::overflow_error("Euler gas arithmetic exceeds float64 range.");
    return x;
}
// Avoid intermediate overflow in products/quotients. A lost nonzero complete
// product is a range error, not a pressure/density floor.
double Product(std::initializer_list<double> factors, std::initializer_list<double> divisors = {}) {
    double m = 1;
    int exponent = 0, e = 0;
    for (double x : factors) {
        if (x == 0)
            return 0;
        m *= std::frexp(x, &e);
        exponent += e;
    }
    for (double x : divisors) {
        m /= std::frexp(x, &e);
        exponent -= e;
    }
    const double result = Checked(std::ldexp(m, exponent));
    if (result == 0)
        throw std::overflow_error("Euler gas nonzero product underflows float64 range.");
    return result;
}
double RootProduct(std::initializer_list<double> factors,
                   std::initializer_list<double> divisors = {}) {
    double m = 1;
    int exponent = 0, e = 0;
    for (double x : factors) {
        m *= std::frexp(x, &e);
        exponent += e;
    }
    for (double x : divisors) {
        m /= std::frexp(x, &e);
        exponent -= e;
    }
    if (exponent % 2 != 0) {
        m *= 2;
        --exponent;
    }
    const double result = Checked(std::ldexp(std::sqrt(m), exponent / 2));
    if (result == 0)
        throw std::overflow_error("Euler gas nonzero square root underflows float64 range.");
    return result;
}
double Normalize(double value, double scale) {
    return Product({value}, {scale});
}
double Upper(double value) {
    return Checked(std::nextafter(Checked(value), std::numeric_limits<double>::infinity()));
}
void Add(double &sum, double &correction, double value) {
    const double next = Checked(sum + value);
    correction = Checked(correction + (std::abs(sum) >= std::abs(value) ? (sum - next) + value
                                                                        : (value - next) + sum));
    sum = next;
}
struct Primitive {
    double u, v, p, c, internal, kinetic;
};
Primitive Decode(double rho, double mx, double my, double energy, double gamma) {
    if (!std::isfinite(rho) || rho <= 0 || !std::isfinite(mx) || !std::isfinite(my) ||
        !std::isfinite(energy) || energy <= 0)
        throw std::invalid_argument(
            "Euler gas requires finite fields and strictly positive density/energy.");
    Primitive q;
    q.u = Product({mx}, {rho});
    q.v = Product({my}, {rho});
    q.kinetic = Checked(Product({.5, mx, mx}, {rho}) + Product({.5, my, my}, {rho}));
    q.internal = Checked(energy - q.kinetic);
    if (q.internal <= 0)
        throw std::invalid_argument("Euler gas stored internal energy must be strictly positive.");
    q.p = Product({gamma - 1, q.internal});
    q.c = RootProduct({gamma, q.p}, {rho});
    return q;
}
Primitive Decode(const EulerGasState &s, std::size_t k, double gamma) {
    return Decode(s.density[k], s.momentumX[k], s.momentumY[k], s.totalEnergy[k], gamma);
}
using Fields = std::array<std::vector<double>, 4>;
std::array<const std::vector<double> *, 4> Arrays(const EulerGasState &s) {
    return {{&s.density, &s.momentumX, &s.momentumY, &s.totalEnergy}};
}
std::array<std::vector<double> *, 4> Arrays(EulerGasState &s) {
    return {{&s.density, &s.momentumX, &s.momentumY, &s.totalEnergy}};
}
EulerGasSummary Summarize(const EulerGasState &s, const EulerGasGridConfig &g, double area) {
    EulerGasSummary result;
    result.minimumDensity = result.maximumDensity = s.density[0];
    const auto n = s.density.size();
    std::array<double, 6> scale{}, sum{}, correction{};
    for (std::size_t k = 0; k < n; ++k) {
        const auto q = Decode(s, k, g.gamma);
        const double values[] = {s.density[k],     s.momentumX[k], s.momentumY[k],
                                 s.totalEnergy[k], q.internal,     q.kinetic};
        for (std::size_t a = 0; a < 6; ++a)
            scale[a] = std::max(scale[a], std::abs(values[a]));
        result.minimumDensity = std::min(result.minimumDensity, s.density[k]);
        result.maximumDensity = std::max(result.maximumDensity, s.density[k]);
        if (k == 0)
            result.minimumPressure = result.maximumPressure = q.p;
        result.minimumPressure = std::min(result.minimumPressure, q.p);
        result.maximumPressure = std::max(result.maximumPressure, q.p);
    }
    double absX = 0, absY = 0, correctionX = 0, correctionY = 0;
    for (std::size_t k = 0; k < n; ++k) {
        const auto q = Decode(s, k, g.gamma);
        const double values[] = {s.density[k],     s.momentumX[k], s.momentumY[k],
                                 s.totalEnergy[k], q.internal,     q.kinetic};
        for (std::size_t a = 0; a < 6; ++a) {
            const double value = scale[a] == 0 ? 0 : Normalize(values[a], scale[a]);
            Add(sum[a], correction[a], value);
            if (a == 1)
                Add(absX, correctionX, std::abs(value));
            if (a == 2)
                Add(absY, correctionY, std::abs(value));
        }
    }
    std::array<double *, 6> output{{&result.mass, &result.momentumX, &result.momentumY,
                                    &result.totalEnergy, &result.internalEnergy,
                                    &result.kineticEnergy}};
    for (std::size_t a = 0; a < 6; ++a)
        *output[a] = Product({Checked(sum[a] + correction[a]), scale[a], area});
    result.absoluteMomentumX = Product({Checked(absX + correctionX), scale[1], area});
    result.absoluteMomentumY = Product({Checked(absY + correctionY), scale[2], area});
    return result;
}
struct Waves {
    double x = 0, y = 0, rate = 0, densityScale = 0, energyScale = 0;
};
Waves Scan(const EulerGasState &s, const EulerGasGridConfig &g,
           std::vector<Primitive> *cache = nullptr) {
    Waves w;
    for (std::size_t k = 0; k < s.density.size(); ++k) {
        const auto q = Decode(s, k, g.gamma);
        if (cache)
            (*cache)[k] = q;
        w.x = std::max(w.x, Upper(Checked(std::abs(q.u) + Upper(q.c))));
        w.y = std::max(w.y, Upper(Checked(std::abs(q.v) + Upper(q.c))));
        w.densityScale = std::max(w.densityScale, s.density[k]);
        w.energyScale = std::max(w.energyScale, s.totalEnergy[k]);
    }
    w.rate =
        Upper(Checked(Upper(Product({w.x}, {g.spacingX})) + Upper(Product({w.y}, {g.spacingY}))));
    return w;
}
double Updated(double old, double increment, double scale) {
    if (increment == 0)
        return old;
    if (std::abs(increment) > 1 && scale > std::numeric_limits<double>::max() / std::abs(increment))
        return Product({Checked(Normalize(old, scale) + increment), scale});
    return Checked(old + Product({increment, scale}));
}
// A face transfer h*F*/spacing, normalized by the shared component scale.
// E+p is split before multiplication so an avoidable sum overflow cannot
// reject a representable CFL-scaled energy transfer.
double Transfer(const EulerGasState &s, const std::vector<Primitive> &q, const Fields &normalized,
                std::size_t left, std::size_t right, std::size_t a, bool x, double h,
                double spacing, double alpha, double scale) {
    double sum = 0, correction = 0;
    for (const auto k : {left, right}) {
        const double velocity = x ? q[k].u : q[k].v;
        if (a == 0)
            Add(sum, correction,
                Product({.5, h, x ? s.momentumX[k] : s.momentumY[k]}, {spacing, scale}));
        else if (a == 1 || a == 2) {
            Add(sum, correction,
                Product({.5, h, a == 1 ? s.momentumX[k] : s.momentumY[k], velocity},
                        {spacing, scale}));
            if ((a == 1) == x)
                Add(sum, correction, Product({.5, h, q[k].p}, {spacing, scale}));
        } else {
            Add(sum, correction, Product({.5, h, s.totalEnergy[k], velocity}, {spacing, scale}));
            Add(sum, correction, Product({.5, h, q[k].p, velocity}, {spacing, scale}));
        }
    }
    Add(sum, correction,
        -Product({.5, h, alpha, Checked(normalized[a][right] - normalized[a][left])}, {spacing}));
    return Checked(sum + correction);
}
void ConservationAudit(EulerGasDiagnostics &d, double factor) {
    d.massDefect = Checked(d.final.mass - d.initial.mass);
    d.momentumXDefect = Checked(d.final.momentumX - d.initial.momentumX);
    d.momentumYDefect = Checked(d.final.momentumY - d.initial.momentumY);
    d.totalEnergyDefect = Checked(d.final.totalEnergy - d.initial.totalEnergy);
    d.massRoundoffAllowance = Product({factor, std::max(d.initial.mass, d.final.mass)});
    d.momentumXRoundoffAllowance =
        Product({factor, std::max(d.initial.absoluteMomentumX, d.final.absoluteMomentumX)});
    d.momentumYRoundoffAllowance =
        Product({factor, std::max(d.initial.absoluteMomentumY, d.final.absoluteMomentumY)});
    d.totalEnergyRoundoffAllowance =
        Product({factor, std::max(d.initial.totalEnergy, d.final.totalEnergy)});
    if (std::abs(d.massDefect) > d.massRoundoffAllowance ||
        std::abs(d.momentumXDefect) > d.momentumXRoundoffAllowance ||
        std::abs(d.momentumYDefect) > d.momentumYRoundoffAllowance ||
        std::abs(d.totalEnergyDefect) > d.totalEnergyRoundoffAllowance)
        throw std::runtime_error("Euler gas stored conservation audit failed.");
}
void Conservation(EulerGasDiagnostics &d, std::size_t n) {
    // Scale-aware heuristic accumulation guard, not an interval error proof.
    const double factor = (64.0 * double(n) + 96.0 * double(d.substeps) + 128.0) *
                          std::numeric_limits<double>::epsilon();
    ConservationAudit(d, factor);
}
} // namespace
} // namespace PhysicsEngine
