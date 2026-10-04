#include "physics/core/periodic_electrostatic_grid.h"
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <utility>

namespace PhysicsEngine {
namespace {
constexpr double MachineEpsilon = std::numeric_limits<double>::epsilon();
double Checked(double x) {
    if (!std::isfinite(x))
        throw std::overflow_error("Electrostatic arithmetic exceeds float64 range.");
    return x;
}
// Exponent-staged complete products avoid needless overflow of scale factors.
double Product(std::initializer_list<double> factors, std::initializer_list<double> divisors = {}) {
    double mantissa = 1;
    int exponent = 0;
    for (double x : factors) {
        Checked(x);
        if (x == 0)
            return 0;
        int e = 0;
        mantissa *= std::frexp(x, &e);
        exponent += e;
    }
    for (double x : divisors) {
        if (!std::isfinite(x) || x == 0)
            throw std::overflow_error("Invalid electrostatic divisor.");
        int e = 0;
        mantissa /= std::frexp(x, &e);
        exponent -= e;
    }
    const double result = Checked(std::ldexp(mantissa, exponent));
    if (result == 0)
        throw std::overflow_error("Nonzero electrostatic arithmetic underflows float64.");
    return result;
}
double Normalized(double x, double scale) {
    return Product({x}, {scale});
}
void Add(double &sum, double &correction, double x) {
    const double next = Checked(sum + x);
    correction =
        Checked(correction + (std::abs(sum) >= std::abs(x) ? (sum - next) + x : (x - next) + sum));
    sum = next;
}
struct Work {
    std::size_t count = 0, limit, n;
    void charge() {
        if (n > limit - count)
            throw std::runtime_error("Electrostatic cell-visit budget exhausted.");
        count += n;
    }
};
double Maximum(const std::vector<double> &a, Work &work) {
    work.charge();
    double scale = 0;
    for (double x : a)
        scale = std::max(scale, std::abs(Checked(x)));
    return scale;
}
struct Summary {
    double mean = 0, meanAbsolute = 0, maximum = 0;
};
Summary Summarize(const std::vector<double> &a, Work &work) {
    Summary result;
    result.maximum = Maximum(a, work);
    work.charge();
    if (result.maximum == 0)
        return result;
    double sum = 0, correction = 0, absolute = 0, absoluteCorrection = 0;
    for (double x : a) {
        const double v = Normalized(x, result.maximum);
        Add(sum, correction, v);
        Add(absolute, absoluteCorrection, std::abs(v));
    }
    result.mean = Product({Checked(sum + correction), result.maximum}, {double(a.size())});
    result.meanAbsolute =
        Product({Checked(absolute + absoluteCorrection), result.maximum}, {double(a.size())});
    return result;
}
double RemoveMean(std::vector<double> &a, Work &work) {
    const double mean = Summarize(a, work).mean;
    work.charge();
    for (double &x : a)
        x = Checked(x - mean);
    return mean;
}
double Rms(const std::vector<double> &a, Work &work) {
    const double scale = Maximum(a, work);
    work.charge();
    if (scale == 0)
        return 0;
    double norm = 0;
    for (double x : a)
        norm = std::hypot(norm, Normalized(x, scale));
    return Product({scale, norm}, {std::sqrt(double(a.size()))});
}
double Inner(const std::vector<double> &a, const std::vector<double> &b, Work &work,
             double weight = 1) {
    work.charge();
    double sa = 0, sb = 0;
    for (std::size_t k = 0; k < a.size(); ++k) {
        sa = std::max(sa, std::abs(Checked(a[k])));
        sb = std::max(sb, std::abs(Checked(b[k])));
    }
    work.charge();
    if (sa == 0 || sb == 0)
        return 0;
    double sum = 0, correction = 0;
    for (std::size_t k = 0; k < a.size(); ++k)
        Add(sum, correction, Product({Normalized(a[k], sa), Normalized(b[k], sb)}));
    return Product({Checked(sum + correction), sa, sb, weight});
}
struct Geometry {
    double area, diagonal, wx, wy;
};
Geometry Derived(const ElectrostaticGridConfig &c) {
    const double ax = Product({1}, {c.spacingX, c.spacingX});
    const double ay = Product({1}, {c.spacingY, c.spacingY});
    const double diagonal = Checked(2 * Checked(ax + ay));
    return {Product({c.spacingX, c.spacingY}), diagonal, Product({ax}, {diagonal}),
            Product({ay}, {diagonal})};
}
std::vector<double> Apply(const std::vector<double> &p, const ElectrostaticGridConfig &c,
                          const Geometry &g, Work &work) {
    work.charge();
    std::vector<double> result(p.size());
    for (std::size_t j = 0; j < c.rows; ++j)
        for (std::size_t i = 0; i < c.columns; ++i) {
            const auto k = i + c.columns * j,
                       left = (i + c.columns - 1) % c.columns + c.columns * j,
                       right = (i + 1) % c.columns + c.columns * j,
                       down = i + c.columns * ((j + c.rows - 1) % c.rows),
                       up = i + c.columns * ((j + 1) % c.rows);
            result[k] = Checked(
                Product({g.wx, Checked(Checked(p[k] - p[left]) + Checked(p[k] - p[right]))}) +
                Product({g.wy, Checked(Checked(p[k] - p[down]) + Checked(p[k] - p[up]))}));
        }
    return result;
}
double Difference(double a, double b, double spacing) {
    return Product({Checked(a - b)}, {spacing});
}
double StoredCharge(const ElectrostaticSnapshot &s, const ElectrostaticGridConfig &c, std::size_t i,
                    std::size_t j) {
    const auto k = i + c.columns * j;
    // Audit epsilon*div(E) directly: div(E) itself need not be representable.
    const auto axis = [&](double a, double b, double spacing) {
        const double difference = a - b;
        if (std::isfinite(difference))
            return Product({c.permittivity, difference}, {spacing});
        // Opposite-sign faces can overflow their difference before scaling.
        return Checked(Product({c.permittivity, a}, {spacing}) -
                       Product({c.permittivity, b}, {spacing}));
    };
    return Checked(
        axis(s.field.xFaces[(i + 1) % c.columns + c.columns * j], s.field.xFaces[k], c.spacingX) +
        axis(s.field.yFaces[i + c.columns * ((j + 1) % c.rows)], s.field.yFaces[k], c.spacingY));
}
void FieldsAndResidual(ElectrostaticSnapshot &s, const ElectrostaticGridConfig &c, Work &work) {
    const auto nx = c.columns, ny = c.rows;
    work.charge();
    for (std::size_t j = 0; j < ny; ++j)
        for (std::size_t i = 0; i < nx; ++i) {
            const auto k = i + nx * j;
            s.field.xFaces[k] =
                -Difference(s.potential[k], s.potential[(i + nx - 1) % nx + nx * j], c.spacingX);
            s.field.yFaces[k] =
                -Difference(s.potential[k], s.potential[i + nx * ((j + ny - 1) % ny)], c.spacingY);
        }
    work.charge();
    for (std::size_t j = 0; j < ny; ++j)
        for (std::size_t i = 0; i < nx; ++i) {
            const auto k = i + nx * j;
            s.gaussResidual[k] = Checked(StoredCharge(s, c, i, j) - s.effectiveCharge[k]);
        }
}
void Audit(ElectrostaticSnapshot &s, const ElectrostaticGridConfig &c, const Geometry &g,
           Work &work) {
    auto &d = s.diagnostics;
    const auto n = s.potential.size(), nx = c.columns, ny = c.rows;
    d.finalGaussRms = Rms(s.gaussResidual, work);
    d.maximumAbsGauss = Maximum(s.gaussResidual, work);
    std::vector<double> originalResidual(n);
    work.charge();
    for (std::size_t j = 0; j < ny; ++j)
        for (std::size_t i = 0; i < nx; ++i) {
            const auto k = i + nx * j;
            originalResidual[k] = Checked(StoredCharge(s, c, i, j) - s.originalCharge[k]);
        }
    d.originalGaussRms = Rms(originalResidual, work);
    d.maximumAbsOriginalGauss = Maximum(originalResidual, work);
    work.charge();
    for (std::size_t j = 0; j < ny; ++j)
        for (std::size_t i = 0; i < nx; ++i) {
            const auto k = i + nx * j;
            s.curl[k] =
                Checked(Difference(s.field.yFaces[k], s.field.yFaces[(i + nx - 1) % nx + nx * j],
                                   c.spacingX) -
                        Difference(s.field.xFaces[k], s.field.xFaces[i + nx * ((j + ny - 1) % ny)],
                                   c.spacingY));
        }
    d.curlRms = Rms(s.curl, work);
    d.maximumAbsCurl = Maximum(s.curl, work);
    d.potentialMean = Summarize(s.potential, work).mean;
    d.meanFieldX = Summarize(s.field.xFaces, work).mean;
    d.meanFieldY = Summarize(s.field.yFaces, work).mean;
    // Accumulate normalized aggregate field norm before squaring: individual
    // squared terms need not be representable if their aggregate energy is.
    work.charge();
    double scale = 0;
    for (std::size_t k = 0; k < n; ++k)
        scale = std::max({scale, std::abs(s.field.xFaces[k]), std::abs(s.field.yFaces[k])});
    work.charge();
    double norm = 0;
    if (scale != 0)
        for (std::size_t k = 0; k < n; ++k) {
            norm = std::hypot(norm, Normalized(s.field.xFaces[k], scale));
            norm = std::hypot(norm, Normalized(s.field.yFaces[k], scale));
        }
    d.fieldEnergy = Product({.5, c.permittivity, g.area, scale, scale, norm, norm});
    d.sourceEnergy = Inner(s.effectiveCharge, s.potential, work, .5 * g.area);
    d.residualEnergyCorrection = Inner(s.gaussResidual, s.potential, work, .5 * g.area);
    d.residualEnergyBound =
        Product({.5, g.area, double(n), d.finalGaussRms, Rms(s.potential, work)});
    d.energyIdentityError =
        Checked(Checked(d.fieldEnergy - d.sourceEnergy) - d.residualEnergyCorrection);
    d.roundoffEnergyAllowance = Product({(32 * double(n) + 128) * MachineEpsilon,
                                         std::max({d.fieldEnergy, std::abs(d.sourceEnergy),
                                                   std::abs(d.residualEnergyCorrection)})});
    if (std::abs(d.energyIdentityError) > d.roundoffEnergyAllowance)
        throw std::runtime_error("Electrostatic stored energy identity fails its roundoff guard.");
}
} // namespace
void ElectrostaticGridConfig::Validate() const {
    if (columns < 2 || rows < 2 || columns > MaximumCells || rows > MaximumCells ||
        columns > MaximumCells / rows)
        throw std::invalid_argument("Periodic electrostatic dimensions exceed supported bounds.");
    if (!std::isfinite(spacingX) || spacingX <= 0 || !std::isfinite(spacingY) || spacingY <= 0 ||
        !std::isfinite(permittivity) || permittivity <= 0)
        throw std::invalid_argument("Electrostatic spacing/permittivity must be positive finite.");
    (void)Derived(*this);
    (void)Product({spacingX, double(columns)});
    (void)Product({spacingY, double(rows)});
}
void ElectrostaticSolveConfig::Validate() const {
    if (!std::isfinite(absoluteGaussTolerance) || absoluteGaussTolerance < 0 ||
        !std::isfinite(relativeGaussTolerance) || relativeGaussTolerance < 0 ||
        maximumIterations > MaximumIterations || maximumCellVisits > MaximumCellVisits)
        throw std::invalid_argument("Electrostatic tolerance or work limit is invalid.");
}
PeriodicElectrostaticGrid::PeriodicElectrostaticGrid(const ElectrostaticGridConfig &config)
    : config_(config) {
    config_.Validate();
    const auto n = config_.columns * config_.rows;
    snapshot_.originalCharge.resize(n);
    snapshot_.effectiveCharge.resize(n);
    snapshot_.potential.resize(n);
    snapshot_.field.xFaces.resize(n);
    snapshot_.field.yFaces.resize(n);
    snapshot_.gaussResidual.resize(n);
    snapshot_.curl.resize(n);
    snapshot_.diagnostics.permittivity = config_.permittivity;
}
ElectrostaticDiagnostics PeriodicElectrostaticGrid::solve(const std::vector<double> &charge,
                                                          const ElectrostaticSolveConfig &options) {
    options.Validate();
    const auto n = config_.columns * config_.rows;
    if (charge.size() != n)
        throw std::invalid_argument("Electrostatic charge size does not match grid.");
    Work work{0, options.maximumCellVisits, n};
    const auto original = Summarize(charge, work); // validates finite entries before staging
    ElectrostaticSnapshot staged;
    staged.originalCharge = charge;
    staged.effectiveCharge = charge;
    staged.potential.resize(n);
    staged.field.xFaces.resize(n);
    staged.field.yFaces.resize(n);
    staged.gaussResidual.resize(n);
    staged.curl.resize(n);
    auto &d = staged.diagnostics;
    d.permittivity = config_.permittivity;
    const auto g = Derived(config_);
    d.originalChargeMean = original.mean;
    d.originalIntegratedCharge = Product({original.mean, double(n), g.area});
    d.neutralityMeanAllowance = Product({64 * MachineEpsilon, original.meanAbsolute});
    if (std::abs(original.mean) > d.neutralityMeanAllowance)
        throw std::invalid_argument(
            "Periodic electrostatics requires neutral charge, within roundoff only.");
    d.removedChargeMean = RemoveMean(staged.effectiveCharge, work);
    const double second = RemoveMean(staged.effectiveCharge, work);
    if (std::abs(second) > d.neutralityMeanAllowance)
        throw std::runtime_error(
            "Electrostatic floating source projection exceeds neutrality allowance.");
    d.removedChargeMean = Checked(d.removedChargeMean + second);
    const auto effective = Summarize(staged.effectiveCharge, work);
    d.effectiveChargeMean = effective.mean;
    if (std::abs(effective.mean) > d.neutralityMeanAllowance)
        throw std::runtime_error(
            "Electrostatic effective source remains nonneutral beyond roundoff.");
    d.effectiveIntegratedCharge = Product({effective.mean, double(n), g.area});
    d.sourceCorrectionAllowance =
        Checked(2 * d.neutralityMeanAllowance + Product({4 * MachineEpsilon, original.maximum}));
    work.charge();
    for (std::size_t k = 0; k < n; ++k)
        d.maximumSourceCorrection = std::max(
            d.maximumSourceCorrection, std::abs(Checked(charge[k] - staged.effectiveCharge[k])));
    if (d.maximumSourceCorrection > d.sourceCorrectionAllowance)
        throw std::runtime_error("Electrostatic source correction exceeds roundoff bound.");
    d.effectiveChargeRms = Rms(staged.effectiveCharge, work);
    d.targetGaussRms = Checked(options.absoluteGaussTolerance +
                               Product({options.relativeGaussTolerance, d.effectiveChargeRms}));
    // A constant residual mean cannot be represented by a periodic divergence.
    if (std::abs(effective.mean) > d.targetGaussRms)
        throw std::runtime_error(
            "Electrostatic effective source mean exceeds requested Gauss tolerance.");
    d.zeroSource = effective.maximum == 0;
    const double sourceScale = d.zeroSource ? 1 : effective.maximum;
    const double potentialScale =
        d.zeroSource ? 1 : Product({sourceScale}, {config_.permittivity, g.diagonal});
    std::vector<double> rhs(n), potential(n), residual(n), direction(n);
    work.charge();
    for (std::size_t k = 0; k < n; ++k)
        rhs[k] = Normalized(staged.effectiveCharge[k], sourceScale);
    RemoveMean(rhs, work);
    residual = rhs;
    direction = rhs;
    double rr = Inner(residual, residual, work);
    bool accepted = false;
    while (true) {
        const double recursiveRms = Product({sourceScale, std::sqrt(rr)}, {std::sqrt(double(n))});
        if (recursiveRms <= d.targetGaussRms || d.iterations == options.maximumIterations) {
            RemoveMean(potential, work);
            work.charge();
            for (std::size_t k = 0; k < n; ++k)
                staged.potential[k] = Product({potential[k], potentialScale});
            FieldsAndResidual(staged, config_, work);
            if (Rms(staged.gaussResidual, work) <= d.targetGaussRms) {
                accepted = true;
                break;
            }
            if (d.iterations == options.maximumIterations)
                break;
            const auto applied = Apply(potential, config_, g, work);
            work.charge();
            for (std::size_t k = 0; k < n; ++k)
                residual[k] = Checked(rhs[k] - applied[k]);
            RemoveMean(residual, work);
            direction = residual;
            rr = Inner(residual, residual, work);
            ++d.residualRestarts;
        }
        if (rr == 0)
            break;
        const auto applied = Apply(direction, config_, g, work);
        const double denominator = Inner(direction, applied, work);
        if (denominator <= 0)
            throw std::runtime_error("Electrostatic CG lost positive definiteness.");
        const double alpha = Product({rr}, {denominator});
        work.charge();
        for (std::size_t k = 0; k < n; ++k) {
            potential[k] = Checked(potential[k] + Product({alpha, direction[k]}));
            residual[k] = Checked(residual[k] - Product({alpha, applied[k]}));
        }
        RemoveMean(residual, work);
        const double next = Inner(residual, residual, work), beta = Product({next}, {rr});
        work.charge();
        for (std::size_t k = 0; k < n; ++k)
            direction[k] = Checked(residual[k] + Product({beta, direction[k]}));
        RemoveMean(direction, work);
        rr = next;
        ++d.iterations;
    }
    if (!accepted)
        throw std::runtime_error("Electrostatic solve did not achieve stored Gauss tolerance.");
    Audit(staged, config_, g, work);
    d.cellVisits = work.count;
    d.hasSolution = true;
    snapshot_ = std::move(staged);
    return snapshot_.diagnostics;
}
} // namespace PhysicsEngine
