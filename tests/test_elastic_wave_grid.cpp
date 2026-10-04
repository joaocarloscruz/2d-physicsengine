#include "catch_amalgamated.hpp"
#include "physics/core/elastic_wave_grid.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <limits>

using namespace PhysicsEngine;
namespace {
constexpr double Pi = 3.1415926535897932384626433832795;
using C = std::complex<double>;
using Amplitude = std::array<C, 5>;
using Matrix = std::array<std::array<C, 5>, 5>;
std::array<std::vector<double> *, 5> Fields(ElasticWaveState &s) {
    return {&s.vx, &s.vy, &s.sigmaXX, &s.sigmaYY, &s.sigmaXY};
}
ElasticWaveState Sample(const ElasticWaveGridConfig &c, int mx, int my, const Amplitude &a) {
    ElasticWaveState s;
    const double ox[] = {0, .5, .5, .5, 0}, oy[] = {.5, 0, .5, .5, 0};
    auto fields = Fields(s);
    for (auto *v : fields)
        v->resize(c.columns * c.rows);
    for (std::size_t j = 0; j < c.rows; ++j)
        for (std::size_t i = 0; i < c.columns; ++i)
            for (int k = 0; k < 5; ++k) {
                const double phase =
                    2 * Pi * (mx * (i + ox[k]) / c.columns + my * (j + oy[k]) / c.rows) + .31;
                (*fields[k])[i + c.columns * j] = std::real(a[k] * std::exp(C(0, phase)));
            }
    return s;
}
double Error(ElasticWaveState a, ElasticWaveState b) {
    auto aa = Fields(a), bb = Fields(b);
    double error = 0;
    for (int k = 0; k < 5; ++k)
        for (std::size_t n = 0; n < aa[k]->size(); ++n)
            error = std::max(error, std::abs((*aa[k])[n] - (*bb[k])[n]));
    return error;
}
void Same(ElasticWaveState a, ElasticWaveState b) {
    auto aa = Fields(a), bb = Fields(b);
    for (int k = 0; k < 5; ++k)
        REQUIRE(*aa[k] == *bb[k]);
}
void Same(const ElasticWaveDiagnostics &a, const ElasticWaveDiagnostics &b) {
    const double aa[] = {a.kineticEnergy,
                         a.strainEnergy,
                         a.totalEnergy,
                         a.modifiedEnergy,
                         a.modifiedEnergyStep,
                         a.physicalEnergyUpperBound,
                         a.meanVx,
                         a.meanVy,
                         a.meanSigmaXX,
                         a.meanSigmaYY,
                         a.meanSigmaXY,
                         a.maxAbsVelocity,
                         a.maxAbsStress,
                         a.meanSigmaZZ,
                         a.maxAbsSigmaZZ,
                         a.compatibilityRms,
                         a.maxAbsCompatibility,
                         a.time,
                         a.stableTimeStep,
                         a.lastSubstep};
    const double bb[] = {b.kineticEnergy,
                         b.strainEnergy,
                         b.totalEnergy,
                         b.modifiedEnergy,
                         b.modifiedEnergyStep,
                         b.physicalEnergyUpperBound,
                         b.meanVx,
                         b.meanVy,
                         b.meanSigmaXX,
                         b.meanSigmaYY,
                         b.meanSigmaXY,
                         b.maxAbsVelocity,
                         b.maxAbsStress,
                         b.meanSigmaZZ,
                         b.maxAbsSigmaZZ,
                         b.compatibilityRms,
                         b.maxAbsCompatibility,
                         b.time,
                         b.stableTimeStep,
                         b.lastSubstep};
    for (unsigned k = 0; k < 20; ++k)
        REQUIRE(aa[k] == bb[k]);
    REQUIRE(a.lastSubsteps == b.lastSubsteps);
    REQUIRE(a.lastCellVisits == b.lastCellVisits);
}
Matrix Identity() {
    Matrix m{};
    for (int k = 0; k < 5; ++k)
        m[k][k] = 1;
    return m;
}
Matrix Multiply(Matrix a, Matrix b) {
    Matrix m{};
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            for (int k = 0; k < 5; ++k)
                m[i][j] += a[i][k] * b[k][j];
    return m;
}
Amplitude Multiply(Matrix a, Amplitude b) {
    Amplitude out{};
    for (int i = 0; i < 5; ++i)
        for (int k = 0; k < 5; ++k)
            out[i] += a[i][k] * b[k];
    return out;
}
Matrix FourierMap(const ElasticWaveGridConfig &c, double x, double y, double h) {
    // Assemble the five-dimensional Fourier generator independently of spatial
    // stencils and the production integrator: i*kappa times Hooke/divergence.
    auto kick = Identity(), drift = Identity();
    const C I(0, 1);
    kick[2][0] = I * (h / 2) * (c.lambda + 2 * c.shearModulus) * x;
    kick[2][1] = I * (h / 2) * c.lambda * y;
    kick[3][0] = I * (h / 2) * c.lambda * x;
    kick[3][1] = I * (h / 2) * (c.lambda + 2 * c.shearModulus) * y;
    kick[4][0] = I * (h / 2) * c.shearModulus * y;
    kick[4][1] = I * (h / 2) * c.shearModulus * x;
    drift[0][2] = I * h * x / c.density;
    drift[0][4] = I * h * y / c.density;
    drift[1][3] = I * h * y / c.density;
    drift[1][4] = I * h * x / c.density;
    return Multiply(kick, Multiply(drift, kick));
}
Amplitude Polarized(const ElasticWaveGridConfig &c, double x, double y, bool shear,
                    double phaseTime, double h, bool continuum = false) {
    const double magnitude = std::hypot(x, y), vx = shear ? -y / magnitude : x / magnitude,
                 vy = shear ? x / magnitude : y / magnitude;
    const double speed = std::sqrt((shear ? c.shearModulus : c.lambda + 2 * c.shearModulus) /
                                   c.density),
                 omega = speed * magnitude;
    const double t = continuum ? omega * phaseTime : phaseTime;
    const double factor =
        std::sin(t) / omega * (continuum ? 1 : std::sqrt(1 - h * h * omega * omega / 4));
    const C I(0, 1);
    return {vx * std::cos(t), vy * std::cos(t),
            I * factor * ((c.lambda + 2 * c.shearModulus) * x * vx + c.lambda * y * vy),
            I * factor * (c.lambda * x * vx + (c.lambda + 2 * c.shearModulus) * y * vy),
            I * factor * c.shearModulus * (y * vx + x * vy)};
}
} // namespace
TEST_CASE("Elastic spatial strain and divergence match an independently assembled negative adjoint",
          "[elastic][operator]") {
    const auto shape = GENERATE(std::pair<std::size_t, std::size_t>{2, 2},
                                std::pair<std::size_t, std::size_t>{2, 5},
                                std::pair<std::size_t, std::size_t>{3, 4});
    ElasticWaveGridConfig c;
    c.columns = shape.first;
    c.rows = shape.second;
    c.spacingX = .23;
    c.spacingY = .41;
    c.density = 2.7;
    ElasticWaveGrid g(c);
    auto s = g.getState();
    auto f = Fields(s);
    const auto n = c.columns * c.rows;
    for (int a = 0; a < 5; ++a)
        for (std::size_t k = 0; k < n; ++k)
            (*f[a])[k] = std::sin(.43 * (k + 1) * (a + 1)) + .07 * a;
    std::vector<std::vector<double>> e(3 * n, std::vector<double>(2 * n));
    for (std::size_t j = 0; j < c.rows; ++j)
        for (std::size_t i = 0; i < c.columns; ++i) {
            const auto k = i + c.columns * j, xp = (i + 1) % c.columns + c.columns * j,
                       yp = i + c.columns * ((j + 1) % c.rows),
                       xm = (i + c.columns - 1) % c.columns + c.columns * j,
                       ym = i + c.columns * ((j + c.rows - 1) % c.rows);
            e[k][xp] += 1 / c.spacingX;
            e[k][k] -= 1 / c.spacingX;
            e[n + k][n + yp] += 1 / c.spacingY;
            e[n + k][n + k] -= 1 / c.spacingY;
            e[2 * n + k][k] += 1 / c.spacingY;
            e[2 * n + k][ym] -= 1 / c.spacingY;
            e[2 * n + k][n + k] += 1 / c.spacingX;
            e[2 * n + k][n + xm] -= 1 / c.spacingX;
        }
    g.setState(s);
    const auto r = g.getSpatialRates();
    const std::vector<double> *er[] = {&r.strainRateXX, &r.strainRateYY, &r.engineeringShearRate};
    const std::vector<double> *ar[] = {&r.accelerationX, &r.accelerationY};
    double power = 0;
    for (std::size_t row = 0; row < 3 * n; ++row) {
        double expected = 0;
        for (std::size_t col = 0; col < 2 * n; ++col)
            expected += e[row][col] * (*f[col / n])[col % n];
        REQUIRE((*er[row / n])[row % n] == Catch::Approx(expected).epsilon(0).margin(3e-15));
        power += (*f[2 + row / n])[row % n] * (*er[row / n])[row % n];
    }
    for (std::size_t col = 0; col < 2 * n; ++col) {
        double expected = 0;
        for (std::size_t row = 0; row < 3 * n; ++row)
            expected -= e[row][col] * (*f[2 + row / n])[row % n] / c.density;
        REQUIRE((*ar[col / n])[col % n] == Catch::Approx(expected).epsilon(0).margin(3e-15));
        power += c.density * (*f[col / n])[col % n] * (*ar[col / n])[col % n];
    }
    REQUIRE(power == Catch::Approx(0).epsilon(0).margin(5e-14));
}
TEST_CASE("Elastic anisotropic two-cell Fourier map agrees with independent five-field matrix",
          "[elastic][oracle]") {
    const auto shape = GENERATE(
        std::pair<std::size_t, std::size_t>{12, 10}, std::pair<std::size_t, std::size_t>{2, 5},
        std::pair<std::size_t, std::size_t>{5, 2}, std::pair<std::size_t, std::size_t>{2, 2});
    ElasticWaveGridConfig c;
    c.columns = shape.first;
    c.rows = shape.second;
    c.spacingX = .23;
    c.spacingY = .41;
    c.density = 2;
    c.lambda = -.5;
    c.shearModulus = 1;
    c.maxSubstep = 10;
    ElasticWaveGrid g(c);
    const double h = .65 * g.getStableTimeStep();
    const double x = 2 * std::sin(Pi / c.columns) / c.spacingX,
                 y = 2 * std::sin(Pi / c.rows) / c.spacingY;
    Amplitude a{C(.7, .2), C(-.4, .3), C(.8, -.1), C(-.6, .2), C(.3, .4)};
    const auto map = FourierMap(c, x, y, h);
    g.setState(Sample(c, 1, 1, a));
    for (int k = 0; k < 37; ++k) {
        g.step(h);
        a = Multiply(map, a);
    }
    REQUIRE(Error(g.getState(), Sample(c, 1, 1, a)) < 8e-14);
    REQUIRE(g.getDiagnostics().lastSubsteps == 1);
    REQUIRE(g.getDiagnostics().lastCellVisits == 4 * c.columns * c.rows);
}
TEST_CASE("Elastic axis and oblique P and S waves follow independent discrete frequencies",
          "[elastic][oracle]") {
    const bool shear = GENERATE(false, true);
    const auto mode =
        GENERATE(std::pair<int, int>{1, 0}, std::pair<int, int>{0, 2}, std::pair<int, int>{2, 3});
    ElasticWaveGridConfig c;
    c.columns = 17;
    c.rows = 13;
    c.spacingX = .2;
    c.spacingY = .37;
    c.density = 2;
    c.lambda = 3;
    c.shearModulus = 2;
    c.maxSubstep = .015;
    ElasticWaveGrid g(c);
    const double x = 2 * std::sin(Pi * mode.first / c.columns) / c.spacingX,
                 y = 2 * std::sin(Pi * mode.second / c.rows) / c.spacingY;
    const double speed = std::sqrt((shear ? 2. : 7.) / 2), omega = speed * std::hypot(x, y),
                 h = c.maxSubstep, phase = 2 * std::asin(h * omega / 2);
    g.setState(Sample(c, mode.first, mode.second, Polarized(c, x, y, shear, 0, h)));
    for (int k = 0; k < 211; ++k)
        g.step(h);
    REQUIRE(Error(g.getState(), Sample(c, mode.first, mode.second,
                                       Polarized(c, x, y, shear, 211 * phase, h))) < 2e-12);
    REQUIRE(g.getCompressionalSpeed() == Catch::Approx(std::sqrt(3.5)).epsilon(0).margin(1e-15));
    REQUIRE(g.getShearSpeed() == 1);
}
TEST_CASE("Elastic continuum P and S errors converge quadratically in axes and oblique directions",
          "[elastic][convergence]") {
    const bool shear = GENERATE(false, true);
    const auto mode =
        GENERATE(std::pair<int, int>{1, 0}, std::pair<int, int>{0, 1}, std::pair<int, int>{1, 2});
    double previous = 0;
    for (int n : {16, 32, 64}) {
        ElasticWaveGridConfig c;
        c.columns = c.rows = n;
        c.spacingX = 2. / n;
        c.spacingY = 3. / n;
        c.density = 2;
        c.lambda = 3;
        c.shearModulus = 2;
        c.maxSubstep = 1;
        ElasticWaveGrid g(c);
        const double x = Pi * mode.first, y = 2 * Pi * mode.second / 3, T = .19;
        const int steps = int(std::ceil(T / (.4 * g.getStableTimeStep())));
        const double h = T / steps;
        g.setState(Sample(c, mode.first, mode.second, Polarized(c, x, y, shear, 0, 0, true)));
        for (int k = 0; k < steps; ++k)
            g.step(h);
        const double error = Error(g.getState(), Sample(c, mode.first, mode.second,
                                                        Polarized(c, x, y, shear, T, 0, true)));
        INFO("n=" << n << " error=" << error << " previous=" << previous);
        if (previous > 0) {
            REQUIRE(error / previous > .18);
            REQUIRE(error / previous < .31);
        }
        previous = error;
    }
}
TEST_CASE("Elastic physical energy oscillates inside the fixed-step invariant envelope",
          "[elastic][energy]") {
    ElasticWaveGridConfig c;
    c.columns = 9;
    c.rows = 7;
    c.spacingX = .2;
    c.spacingY = .35;
    c.density = 4;
    c.lambda = 2;
    c.shearModulus = 3;
    c.maxSubstep = 10;
    ElasticWaveGrid g(c);
    auto s = g.getState();
    auto f = Fields(s);
    for (int a = 0; a < 5; ++a)
        for (std::size_t k = 0; k < f[a]->size(); ++k)
            (*f[a])[k] = std::sin(.43 * (k + 1) * (a + 1)) + .07 * a;
    g.setState(s);
    const double h = .7 * g.getStableTimeStep(), invariant = g.getModifiedEnergy(h),
                 initial = g.getDiagnostics().totalEnergy;
    const auto compatibility = g.getCompatibility();
    const auto before = g.getDiagnostics();
    double low = initial, high = initial;
    for (int k = 0; k < 400; ++k) {
        g.step(h);
        const auto d = g.getDiagnostics();
        REQUIRE(d.modifiedEnergy == Catch::Approx(invariant).epsilon(2e-12));
        REQUIRE(d.totalEnergy >= invariant * (1 - 2e-12));
        REQUIRE(d.totalEnergy <= d.physicalEnergyUpperBound * (1 + 2e-12));
        low = std::min(low, d.totalEnergy);
        high = std::max(high, d.totalEnergy);
    }
    REQUIRE(high - low > 1e-5 * initial);
    const auto after = g.getDiagnostics();
    REQUIRE(after.meanVx == Catch::Approx(before.meanVx).epsilon(0).margin(3e-14));
    REQUIRE(after.meanVy == Catch::Approx(before.meanVy).epsilon(0).margin(3e-14));
    REQUIRE(after.meanSigmaXX == Catch::Approx(before.meanSigmaXX).epsilon(0).margin(3e-14));
    REQUIRE(after.meanSigmaYY == Catch::Approx(before.meanSigmaYY).epsilon(0).margin(3e-14));
    REQUIRE(after.meanSigmaXY == Catch::Approx(before.meanSigmaXY).epsilon(0).margin(3e-14));
    const auto defect = g.getCompatibility();
    for (std::size_t k = 0; k < defect.size(); ++k)
        REQUIRE(defect[k] == Catch::Approx(compatibility[k]).epsilon(0).margin(2e-11));
}
TEST_CASE("Elastic DC translation and prestress retain independent plane-strain energy",
          "[elastic][dc]") {
    ElasticWaveGridConfig c;
    c.columns = 2;
    c.rows = 3;
    c.spacingX = .2;
    c.spacingY = .4;
    c.density = 4;
    c.lambda = 2;
    c.shearModulus = 3;
    ElasticWaveGrid g(c);
    auto s = g.getState();
    s.vx.assign(6, 2);
    s.vy.assign(6, -3);
    s.sigmaXX.assign(6, 4);
    s.sigmaYY.assign(6, 5);
    s.sigmaXY.assign(6, 6);
    g.setState(s);
    g.step(.037);
    Same(g.getState(), s);
    const auto d = g.getDiagnostics();
    // Independent constitutive energy from the represented inputs. The native
    // weighted hypot reduction has O(N) rounded operations; a two-ULP absolute
    // decimal tolerance was tighter than GCC/libm's valid reduction error.
    const auto n = s.vx.size();
    const long double volume = static_cast<long double>(n) * c.spacingX * c.spacingY;
    const double kinetic = static_cast<double>(volume * c.density / 2 * (2 * 2 + 3 * 3));
    const double strain = static_cast<double>(volume *
        (81.L / (8 * (static_cast<long double>(c.lambda) + c.shearModulus)) +
         1.L / (8 * c.shearModulus) + 36.L / (2 * c.shearModulus)));
    const double roundoff = (8.0 * n + 16) * std::numeric_limits<double>::epsilon();
    REQUIRE(d.kineticEnergy == Catch::Approx(kinetic).epsilon(0).margin(roundoff * kinetic));
    REQUIRE(d.strainEnergy == Catch::Approx(strain).epsilon(0).margin(roundoff * strain));
    REQUIRE(d.modifiedEnergy == d.totalEnergy);
    REQUIRE(d.compatibilityRms == 0);
    REQUIRE(d.meanSigmaZZ == Catch::Approx(1.8).epsilon(0).margin(1e-15));
    for (double z : g.getOutOfPlaneStress())
        REQUIRE(z == Catch::Approx(1.8).epsilon(0).margin(1e-15));
}
TEST_CASE("Elastic compression and engineering shear have separate Hooke responses",
          "[elastic][operator]") {
    const bool shear = GENERATE(false, true);
    ElasticWaveGridConfig c;
    c.columns = 8;
    c.rows = 6;
    c.lambda = 2;
    c.shearModulus = 3;
    ElasticWaveGrid g(c);
    g.setState(
        Sample(c, 1, 0, shear ? Amplitude{0., 1., 0., 0., 0.} : Amplitude{1., 0., 0., 0., 0.}));
    g.step(.03);
    const auto s = g.getState();
    for (std::size_t k = 0; k < s.vx.size(); ++k) {
        if (shear) {
            REQUIRE(s.vx[k] == 0);
            REQUIRE(s.sigmaXX[k] == 0);
            REQUIRE(s.sigmaYY[k] == 0);
        } else {
            REQUIRE(s.vy[k] == 0);
            REQUIRE(s.sigmaXY[k] == 0);
            REQUIRE(s.sigmaYY[k] == Catch::Approx(.25 * s.sigmaXX[k]).epsilon(0).margin(2e-17));
        }
    }
}
TEST_CASE("Elastic finite-range validation and transactional failures retain snapshots",
          "[elastic][validation]") {
    ElasticWaveGridConfig c;
    c.columns = c.rows = 2;
    ElasticWaveGrid g(c);
    auto s = g.getState();
    s.vx.assign(4, 1);
    g.setState(s);
    g.step(.01);
    const auto old = g.getState();
    const auto d = g.getDiagnostics();
    auto bad = old;
    bad.sigmaXY.pop_back();
    REQUIRE_THROWS_AS(g.setState(bad), std::invalid_argument);
    bad = old;
    bad.vy[0] = std::numeric_limits<double>::infinity();
    REQUIRE_THROWS_AS(g.setState(bad), std::invalid_argument);
    bad = old;
    bad.sigmaXX[0] = std::numeric_limits<double>::max();
    REQUIRE_THROWS_AS(g.setState(bad), std::overflow_error);
    REQUIRE_THROWS_AS(g.step(-1), std::invalid_argument);
    REQUIRE_THROWS_AS(g.step(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
    REQUIRE_THROWS_AS(g.step(1e10), std::length_error);
    REQUIRE_THROWS_AS(g.step(std::numeric_limits<double>::denorm_min()), std::overflow_error);
    REQUIRE_THROWS_AS(g.getModifiedEnergy(1), std::invalid_argument);
    g.step(0);
    Same(g.getState(), old);
    Same(g.getDiagnostics(), d);
    auto tiny = g.getState();
    for (auto *f : Fields(tiny))
        f->assign(4, 0);
    tiny.vx[0] = 1e-200;
    REQUIRE_THROWS_AS(g.setState(tiny), std::overflow_error);
    Same(g.getState(), old);
    Same(g.getDiagnostics(), d);
}
TEST_CASE("Elastic finite energy can still reject an overflowing raw stencil difference atomically",
          "[elastic][validation]") {
    ElasticWaveGridConfig c;
    c.columns = c.rows = 2;
    c.spacingX = c.spacingY = 1e-8;
    c.density = c.shearModulus = 1e300;
    c.lambda = -.5e300;
    ElasticWaveGrid g(c);
    auto s = g.getState();
    s.sigmaXX = {1e308, -1e308, 1e308, -1e308};
    g.setState(s);
    const auto d = g.getDiagnostics();
    REQUIRE(std::isfinite(d.totalEnergy));
    REQUIRE_THROWS_AS(g.step(.5 * g.getStableTimeStep()), std::overflow_error);
    Same(g.getState(), s);
    Same(g.getDiagnostics(), d);
}
TEST_CASE("Elastic geometry material derived-range and work limits precede allocations and updates",
          "[elastic][validation]") {
    const int mode = GENERATE(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12);
    ElasticWaveGridConfig c;
    if (mode == 0)
        c.columns = 1;
    if (mode == 1)
        c.rows = std::numeric_limits<std::size_t>::max();
    if (mode == 2)
        c.spacingX = 0;
    if (mode == 3)
        c.spacingY = 1e200;
    if (mode == 4)
        c.density = -1;
    if (mode == 5)
        c.shearModulus = 0;
    if (mode == 6)
        c.lambda = -.8;
    if (mode == 7)
        c.lambda = std::numeric_limits<double>::quiet_NaN();
    if (mode == 8)
        c.cflSafety = 1;
    if (mode == 9)
        c.maximumSubsteps = ElasticWaveGridConfig::MaximumSubsteps + 1;
    if (mode == 10)
        c.maximumCellVisits = 0;
    if (mode == 11)
        c.maxSubstep = 0;
    if (mode == 12)
        c.columns = c.rows = 513;
    REQUIRE_THROWS(c.Validate());
    REQUIRE_THROWS(ElasticWaveGrid(c));
    ElasticWaveGridConfig good;
    good.columns = good.rows = 2;
    good.maximumCellVisits = 16;
    good.maximumSubsteps = 1;
    ElasticWaveGrid g(good);
    auto before = g.getState();
    auto d = g.getDiagnostics();
    REQUIRE_THROWS_AS(g.step(2 * g.getStableTimeStep()), std::length_error);
    Same(g.getState(), before);
    Same(g.getDiagnostics(), d);
    good.maximumCellVisits = 15;
    ElasticWaveGrid limited(good);
    REQUIRE_THROWS_AS(limited.step(.01), std::length_error);
}
TEST_CASE("Elastic auxetic staged modulus clock and ownership semantics remain representable",
          "[elastic][validation]") {
    ElasticWaveGridConfig c;
    c.columns = c.rows = 2;
    c.density = c.shearModulus = 1e308;
    c.lambda = -6e307;
    ElasticWaveGrid large(c);
    REQUIRE(large.getCompressionalSpeed() ==
            Catch::Approx(std::sqrt(1.4)).epsilon(0).margin(3e-16));
    auto small = c;
    small.density = 1;
    small.lambda = small.shearModulus = 1e-40;
    small.maxSubstep = 1e14;
    ElasticWaveGrid clock(small);
    clock.step(1e14);
    auto state = clock.getState();
    auto d = clock.getDiagnostics();
    REQUIRE_THROWS_AS(clock.step(1e-10), std::overflow_error);
    Same(clock.getState(), state);
    Same(clock.getDiagnostics(), d);
    ElasticWaveGrid a, b;
    auto s = a.getState();
    s.vx[0] = .7;
    s.sigmaXY[1] = -.2;
    a.setState(s);
    b.setState(s);
    for (double t : {.021, 0., .137, .004}) {
        a.step(t);
        b.step(t);
    }
    Same(a.getState(), b.getState());
    Same(a.getDiagnostics(), b.getDiagnostics());
    auto snapshot = a.getState();
    snapshot.vx[0] = 100;
    REQUIRE(a.getState().vx[0] != 100);
    auto config = a.getConfig();
    config.lambda = 100;
    REQUIRE(a.getConfig().lambda != 100);
    auto rates = a.getSpatialRates();
    rates.accelerationX[0] = 100;
    REQUIRE(a.getSpatialRates().accelerationX[0] != 100);
    auto saved = a.getState();
    a.setState(saved);
    REQUIRE(a.getDiagnostics().time == b.getDiagnostics().time);
    REQUIRE(a.getDiagnostics().lastSubsteps == 0);
    REQUIRE(a.getDiagnostics().modifiedEnergyStep == 0);
}

TEST_CASE("Elastic aggregate subnormal energies preserve constant stored means",
          "[elastic][range]") {
    ElasticWaveGridConfig c;
    c.columns = c.rows = 512;
    c.density = 1e308;
    ElasticWaveGrid g(c);
    auto s = g.getState();
    const double value = 7e-319;
    std::fill(s.vx.begin(), s.vx.end(), value);
    g.setState(s);
    const auto d = g.getDiagnostics();
    INFO("stored=" << value << " reported=" << d.meanVx);
    // Independent aggregate: sqrt(N*rho/2)*v avoids per-particle energy underflow.
    const double rootAggregate = std::sqrt(double(s.vx.size())) * std::sqrt(c.density / 2) * value;
    REQUIRE(rootAggregate * rootAggregate == std::numeric_limits<double>::denorm_min());
    REQUIRE(d.kineticEnergy == rootAggregate * rootAggregate);
    REQUIRE(d.meanVx == value);
    g.step(.01);
    REQUIRE(g.getDiagnostics().meanVx == value);
    REQUIRE(g.getDiagnostics().lastCellVisits == 4 * s.vx.size());

    std::fill(s.vy.begin(), s.vy.end(), -value);
    g.setState(s);
    REQUIRE(g.getDiagnostics().meanVx == value);
    REQUIRE(g.getDiagnostics().meanVy == -value);

    c.density = 1;
    c.lambda = c.shearModulus = 1e-308;
    ElasticWaveGrid stress(c);
    auto t = stress.getState();
    std::fill(t.sigmaXX.begin(), t.sigmaXX.end(), value);
    std::fill(t.sigmaYY.begin(), t.sigmaYY.end(), value);
    std::fill(t.sigmaXY.begin(), t.sigmaXY.end(), value);
    stress.setState(t);
    const auto sd = stress.getDiagnostics();
    const double stressRootAggregate =
        std::sqrt(double(t.sigmaXX.size())) * value *
        std::hypot(std::sqrt(.5 / (c.lambda + c.shearModulus)), std::sqrt(.5 / c.shearModulus));
    REQUIRE(stressRootAggregate * stressRootAggregate ==
            2 * std::numeric_limits<double>::denorm_min());
    REQUIRE(sd.strainEnergy == stressRootAggregate * stressRootAggregate);
    REQUIRE(sd.meanSigmaXX == value);
    REQUIRE(sd.meanSigmaYY == value);
    REQUIRE(sd.meanSigmaXY == value);
    REQUIRE(sd.meanSigmaZZ == stress.getOutOfPlaneStress()[0]);
    REQUIRE(sd.meanSigmaZZ ==
            Catch::Approx(.5 * value).epsilon(0).margin(std::numeric_limits<double>::denorm_min()));
}
TEST_CASE("Elastic means retain signed cancellation and reject erased dynamic ranges",
          "[elastic][range]") {
    ElasticWaveGridConfig c;
    c.columns = c.rows = 2;
    ElasticWaveGrid g(c);
    auto s = g.getState();
    s.vx = {1e15, 1, -1e15, 3};
    s.vy = {-1e15, -1, 1e15, -3};
    s.sigmaXX = s.vx;
    s.sigmaYY = s.vy;
    s.sigmaXY = s.vx;
    g.setState(s);
    auto d = g.getDiagnostics();
    REQUIRE(d.meanVx == Catch::Approx(1).epsilon(0).margin(3e-16));
    REQUIRE(d.meanVy == Catch::Approx(-1).epsilon(0).margin(3e-16));
    REQUIRE(d.meanSigmaXX == Catch::Approx(1).epsilon(0).margin(3e-16));
    REQUIRE(d.meanSigmaYY == Catch::Approx(-1).epsilon(0).margin(3e-16));
    REQUIRE(d.meanSigmaXY == Catch::Approx(1).epsilon(0).margin(3e-16));
    REQUIRE(d.meanSigmaZZ == 0);
    const auto before = g.getState();
    s.vx = {1e100, 1e-300, -1e100, 0};
    REQUIRE_THROWS_AS(g.setState(s), std::overflow_error);
    Same(g.getState(), before);
    Same(g.getDiagnostics(), d);
    // Same erased contribution, encountered before accumulator rescaling.
    s.vx = {1e-300, 1e100, -1e100, 0};
    REQUIRE_THROWS_AS(g.setState(s), std::overflow_error);
    Same(g.getState(), before);
    Same(g.getDiagnostics(), d);
}
