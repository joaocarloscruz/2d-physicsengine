#include "euler_gas_numeric.h"

namespace PhysicsEngine {
namespace {
constexpr std::size_t MaximumSlopeHalvings = 32;
struct Work {
    EulerGasSecondOrderDiagnostics &d;
    const EulerGasSecondOrderConfig &options;
    std::size_t n;
    void Charge(std::size_t count, bool reserveFinal = true) {
        const auto remaining = options.maximumCellVisits - d.cellVisits;
        const auto reserve = reserveFinal ? 2 * n : 0;
        if (remaining < reserve || count > remaining - reserve)
            throw std::runtime_error("Second-order Euler gas cell-visit budget exhausted.");
        d.cellVisits += count;
    }
};
struct FacePrimitive {
    std::vector<double> u, v, p;
};
struct Reconstruction {
    Fields center, delta, correction;
    std::array<Fields, 4> face; // x-, x+, y-, y+; normalized conserved face states
    std::array<FacePrimitive, 4> primitive;
    explicit Reconstruction(std::size_t n) {
        for (auto *fields : {&center, &delta, &correction})
            for (auto &values : *fields)
                values.resize(n);
        for (auto &fields : face)
            for (auto &values : fields)
                values.resize(n);
        for (auto &q : primitive) {
            q.u.resize(n);
            q.v.resize(n);
            q.p.resize(n);
        }
    }
};
using Scale = std::array<double, 4>;
struct Prepared {
    Waves waves;
    Scale scale;
};
double MC(double left, double right) {
    if (left == 0 || right == 0 || std::signbit(left) != std::signbit(right))
        return 0;
    const double centered = Product({.5, Checked(left + right)});
    return std::copysign(std::min({std::abs(2 * left), std::abs(centered), std::abs(2 * right)}),
                         left);
}
struct Point {
    std::array<double, 4> normalized;
    Primitive primitive;
};
bool Trial(const EulerGasState &s, std::size_t k, double gamma, const Scale &scale,
           const std::array<double, 4> &sx, const std::array<double, 4> &sy, double theta,
           const Reconstruction &reconstruction, std::array<Point, 4> &faces, bool &rangeFailure) {
    const auto fields = Arrays(s);
    // Check every used face midpoint and all four corners. One scalar theta
    // preserves the original mean and both opposing-face averages.
    constexpr int coordinates[8][2] = {{-1, 0},  {1, 0},  {0, -1}, {0, 1},
                                       {-1, -1}, {1, -1}, {-1, 1}, {1, 1}};
    try {
        for (std::size_t point = 0; point < 8; ++point) {
            std::array<double, 4> value{};
            for (std::size_t a = 0; a < 4; ++a) {
                // theta=0 copies the original stored center exactly, including
                // subnormal components; never normalize/rescale the fallback.
                value[a] = (*fields[a])[k];
                if (theta != 0) {
                    const double slope =
                        Checked(coordinates[point][0] * sx[a] + coordinates[point][1] * sy[a]);
                    const double increment = Product({.5, theta, slope});
                    value[a] = Updated(value[a], increment, scale[a]);
                }
            }
            const auto q = Decode(value[0], value[1], value[2], value[3], gamma);
            if (point < 4) {
                faces[point].primitive = q;
                for (std::size_t a = 0; a < 4; ++a)
                    faces[point].normalized[a] =
                        theta == 0 ? reconstruction.center[a][k] : Normalize(value[a], scale[a]);
            }
        }
        return true;
    } catch (const std::invalid_argument &) {
        return false;
    } catch (const std::overflow_error &) {
        rangeFailure = true;
        return false;
    }
}
Prepared Prepare(const EulerGasState &s, const EulerGasGridConfig &g, Reconstruction &r,
                 Work &work) {
    const auto n = s.density.size(), nx = g.columns, ny = g.rows;
    Prepared result;
    work.Charge(n);
    for (std::size_t k = 0; k < n; ++k) {
        (void)Decode(s, k, g.gamma);
        result.waves.densityScale = std::max(result.waves.densityScale, s.density[k]);
        result.waves.energyScale = std::max(result.waves.energyScale, s.totalEnergy[k]);
    }
    const double momentumScale = RootProduct({result.waves.densityScale, result.waves.energyScale});
    result.scale = {
        {result.waves.densityScale, momentumScale, momentumScale, result.waves.energyScale}};
    const auto fields = Arrays(s);
    work.Charge(n);
    for (std::size_t k = 0; k < n; ++k)
        for (std::size_t a = 0; a < 4; ++a)
            r.center[a][k] = Normalize((*fields[a])[k], result.scale[a]);
    for (std::size_t j = 0; j < ny; ++j)
        for (std::size_t i = 0; i < nx; ++i) {
            const auto k = i + nx * j, left = (i + nx - 1) % nx + nx * j,
                       right = (i + 1) % nx + nx * j;
            const auto bottom = i + nx * ((j + ny - 1) % ny), top = i + nx * ((j + 1) % ny);
            work.Charge(1);
            ++work.d.reconstructionTrials; // Includes this cell's MC slope work.
            std::array<double, 4> sx{}, sy{};
            bool limited = false;
            for (std::size_t a = 0; a < 4; ++a) {
                const double dl = Checked(r.center[a][k] - r.center[a][left]),
                             dr = Checked(r.center[a][right] - r.center[a][k]);
                const double db = Checked(r.center[a][k] - r.center[a][bottom]),
                             dt = Checked(r.center[a][top] - r.center[a][k]);
                sx[a] = MC(dl, dr);
                sy[a] = MC(db, dt);
                // Compare twice the chosen slope, so a diagnostic-only
                // half-gradient cannot introduce a spurious underflow reject.
                limited = limited || 2 * sx[a] != Checked(dl + dr) || 2 * sy[a] != Checked(db + dt);
            }
            if (limited)
                ++work.d.limitedSlopeCells;
            std::array<Point, 4> faces;
            double theta = 1;
            bool rangeFailure = false;
            for (std::size_t trial = 0;; ++trial) {
                if (trial != 0) {
                    work.Charge(1);
                    ++work.d.reconstructionTrials;
                }
                if (Trial(s, k, g.gamma, result.scale, sx, sy, theta, r, faces, rangeFailure))
                    break;
                if (theta == 0)
                    throw std::runtime_error(
                        "Second-order Euler gas center fallback is inadmissible.");
                theta = trial == MaximumSlopeHalvings ? 0 : .5 * theta;
            }
            if (rangeFailure)
                ++work.d.rangeLimitedCells;
            if (theta < 1)
                ++work.d.positivityLimitedCells;
            if (theta == 0)
                ++work.d.zeroSlopeFallbackCells;
            work.d.minimumSlopeScale = std::min(work.d.minimumSlopeScale, theta);
            for (std::size_t face = 0; face < 4; ++face) {
                for (std::size_t a = 0; a < 4; ++a)
                    r.face[face][a][k] = faces[face].normalized[a];
                const auto &q = faces[face].primitive;
                r.primitive[face].u[k] = q.u;
                r.primitive[face].v[k] = q.v;
                r.primitive[face].p[k] = q.p;
                if (face < 2)
                    result.waves.x =
                        std::max(result.waves.x, Upper(Checked(std::abs(q.u) + Upper(q.c))));
                else
                    result.waves.y =
                        std::max(result.waves.y, Upper(Checked(std::abs(q.v) + Upper(q.c))));
            }
        }
    result.waves.rate = Upper(Checked(Upper(Product({result.waves.x}, {g.spacingX})) +
                                      Upper(Product({result.waves.y}, {g.spacingY}))));
    ++work.d.reconstructionPreparations;
    work.d.maximumSignalSpeedX = std::max(work.d.maximumSignalSpeedX, result.waves.x);
    work.d.maximumSignalSpeedY = std::max(work.d.maximumSignalSpeedY, result.waves.y);
    return result;
}
double Limit(double rate, double safety) {
    // Complete safety/(2*rate), permitting an infinite duration bound when it
    // exceeds float64. Neither 2*rate nor safety/2 is formed prematurely.
    int er = 0, es = 0;
    const double m = .5 * std::frexp(safety, &es) / std::frexp(rate, &er);
    const double limit = std::ldexp(m, es - er);
    if (std::isinf(limit))
        return limit;
    const double down = std::nextafter(limit, 0.0);
    if (down <= 0)
        throw std::overflow_error("Second-order Euler gas stable duration is unrepresentable.");
    return down;
}
double FaceTransfer(const Reconstruction &r, std::size_t fl, std::size_t left, std::size_t fr,
                    std::size_t right, std::size_t a, bool x, double h, double spacing,
                    double alpha, const Scale &scale) {
    double sum = 0, correction = 0;
    const std::size_t normal = x ? 1 : 2;
    for (const auto endpoint : {std::pair<std::size_t, std::size_t>{fl, left}, {fr, right}}) {
        const auto face = endpoint.first, k = endpoint.second;
        const double velocity = x ? r.primitive[face].u[k] : r.primitive[face].v[k];
        if (a == 0)
            Add(sum, correction,
                Product({.5, h, r.face[face][normal][k], scale[normal]}, {spacing, scale[0]}));
        else if (a == 1 || a == 2) {
            Add(sum, correction, Product({.5, h, r.face[face][a][k], velocity}, {spacing}));
            if (a == normal)
                Add(sum, correction, Product({.5, h, r.primitive[face].p[k]}, {spacing, scale[a]}));
        } else {
            Add(sum, correction, Product({.5, h, r.face[face][3][k], velocity}, {spacing}));
            Add(sum, correction,
                Product({.5, h, r.primitive[face].p[k], velocity}, {spacing, scale[3]}));
        }
    }
    Add(sum, correction,
        -Product({.5, h, alpha, Checked(r.face[fr][a][right] - r.face[fl][a][left])}, {spacing}));
    return Checked(sum + correction);
}
void Forward(const EulerGasState &source, EulerGasState &target, const EulerGasGridConfig &g,
             const Prepared &prepared, Reconstruction &r, Work &work, double h) {
    const auto n = source.density.size(), nx = g.columns, ny = g.rows;
    work.Charge(n);
    for (std::size_t k = 0; k < n; ++k)
        for (std::size_t a = 0; a < 4; ++a)
            r.delta[a][k] = r.correction[a][k] = 0;
    for (const bool x : {true, false}) {
        work.Charge(n);
        for (std::size_t j = 0; j < ny; ++j)
            for (std::size_t i = 0; i < nx; ++i) {
                const auto right = i + nx * j;
                const auto left = x ? (i + nx - 1) % nx + nx * j : i + nx * ((j + ny - 1) % ny);
                for (std::size_t a = 0; a < 4; ++a) {
                    const double transfer = FaceTransfer(
                        r, x ? 1 : 3, left, x ? 0 : 2, right, a, x, h, x ? g.spacingX : g.spacingY,
                        x ? prepared.waves.x : prepared.waves.y, prepared.scale);
                    Add(r.delta[a][left], r.correction[a][left], -transfer);
                    Add(r.delta[a][right], r.correction[a][right], transfer);
                }
            }
    }
    const auto input = Arrays(source);
    auto output = Arrays(target);
    work.Charge(n);
    for (std::size_t k = 0; k < n; ++k) {
        for (std::size_t a = 0; a < 4; ++a)
            (*output[a])[k] = Updated((*input[a])[k], Checked(r.delta[a][k] + r.correction[a][k]),
                                      prepared.scale[a]);
        (void)Decode(target, k, g.gamma);
    }
    ++work.d.forwardEulerStages;
}
void Blend(const EulerGasState &old, EulerGasState &candidate, const EulerGasGridConfig &g,
           const Scale &scale, Work &work) {
    const auto input = Arrays(old);
    auto output = Arrays(candidate);
    work.Charge(old.density.size());
    for (std::size_t k = 0; k < old.density.size(); ++k) {
        for (std::size_t a = 0; a < 4; ++a) {
            const double initial = (*input[a])[k], evolved = (*output[a])[k];
            (*output[a])[k] =
                initial == evolved
                    ? initial
                    : Product({.5,
                               Checked(Normalize(initial, scale[a]) + Normalize(evolved, scale[a])),
                               scale[a]});
        }
        (void)Decode(candidate, k, g.gamma);
    }
    ++work.d.blendPasses;
}
} // namespace

void EulerGasSecondOrderConfig::Validate() const {
    EulerGasStepConfig::Validate();
    if (maximumAttempts > MaximumAttempts || maximumRetriesPerSubstep > MaximumRetriesPerSubstep)
        throw std::invalid_argument("Second-order Euler gas retry budgets exceed hard ceilings.");
}
EulerGasSecondOrderDiagnostics
PeriodicEulerGasGrid::stepSecondOrder(double duration, const EulerGasSecondOrderConfig &options) {
    options.Validate();
    if (!std::isfinite(duration) || duration < 0)
        throw std::invalid_argument(
            "Second-order Euler gas duration must be finite and nonnegative.");
    const auto n = state_.density.size();
    if (options.maximumCellVisits / n < (duration == 0 ? 3u : 19u))
        throw std::runtime_error("Second-order Euler gas minimum cell-visit budget exhausted.");
    if (duration > 0 && (options.maximumSubsteps == 0 || options.maximumAttempts == 0))
        throw std::runtime_error("Second-order Euler gas attempt/substep budget exhausted.");
    EulerGasSecondOrderDiagnostics d;
    d.duration = duration;
    d.timeBefore = time_;
    d.timeAfter = Checked(time_ + duration);
    if (duration > 0 && d.timeAfter <= time_)
        throw std::overflow_error("Second-order Euler gas clock increment is unrepresentable.");
    const double area = Product({config_.spacingX, config_.spacingY});
    d.initial = Summarize(state_, config_, area);
    d.cellVisits = 2 * n;
    if (duration == 0) {
        const auto waves = Scan(state_, config_);
        d.cellVisits += n;
        d.maximumSignalSpeedX = waves.x;
        d.maximumSignalSpeedY = waves.y;
        d.final = d.initial;
        d.zeroDurationNoOp = true;
        Conservation(d, n);
        diagnostics_ = d;
        secondOrderDiagnostics_ = d;
        return d;
    }
    Work work{d, options, n};
    auto staged = state_;
    EulerGasState candidate;
    for (auto *field : Arrays(candidate))
        field->resize(n);
    Reconstruction r(n);
    double elapsed = 0, remaining = duration;
    while (remaining > 0) {
        if (d.substeps == options.maximumSubsteps)
            throw std::runtime_error("Second-order Euler gas substep budget exhausted.");
        double attemptedDuration = remaining;
        std::size_t retries = 0;
        while (true) {
            if (d.attempts == options.maximumAttempts)
                throw std::runtime_error("Second-order Euler gas attempt budget exhausted.");
            ++d.attempts;
            const auto first = Prepare(staged, config_, r, work);
            const double h = std::min({attemptedDuration, options.maxSubstep,
                                       Limit(first.waves.rate, options.cflSafety)});
            const double cflFirst = Product({h, first.waves.rate});
            if (Product({2, h, first.waves.rate}) > options.cflSafety || cflFirst >= .5)
                throw std::runtime_error("Second-order Euler gas first-stage rounded CFL failed.");
            const double nextElapsed = h == remaining ? duration : Checked(elapsed + h);
            if (nextElapsed <= elapsed)
                throw std::overflow_error("Second-order Euler gas represented duration stagnates.");
            Forward(staged, candidate, config_, first, r, work, h);
            const auto second = Prepare(candidate, config_, r, work);
            const double cflSecond = Product({h, second.waves.rate});
            if (h > Limit(second.waves.rate, options.cflSafety) ||
                Product({2, h, second.waves.rate}) > options.cflSafety || cflSecond >= .5) {
                ++d.rejectedAttempts;
                d.maximumRejectedCfl = std::max(d.maximumRejectedCfl, cflSecond);
                if (retries == options.maximumRetriesPerSubstep)
                    throw std::runtime_error(
                        "Second-order Euler gas stage-CFL retry budget exhausted.");
                ++retries;
                attemptedDuration =
                    std::min(Product({.5, h}), Limit(second.waves.rate, options.cflSafety));
                continue;
            }
            Forward(candidate, candidate, config_, second, r, work, h);
            Blend(staged, candidate, config_, first.scale, work);
            std::swap(staged, candidate);
            ++d.substeps;
            d.lastSubstep = h;
            d.maximumCfl = std::max({d.maximumCfl, cflFirst, cflSecond});
            elapsed = nextElapsed;
            remaining = h == remaining ? 0 : Checked(duration - elapsed);
            if (remaining < 0)
                throw std::overflow_error("Second-order Euler gas duration loses range.");
            break;
        }
    }
    work.Charge(2 * n, false);
    d.final = Summarize(staged, config_, area);
    const double factor = (64.0 * double(n) + 288.0 * double(d.substeps) + 128.0) *
                          std::numeric_limits<double>::epsilon();
    ConservationAudit(d, factor);
    state_ = std::move(staged);
    time_ = d.timeAfter;
    diagnostics_ = d;
    secondOrderDiagnostics_ = d;
    return d;
}
} // namespace PhysicsEngine
