#include "euler_gas_oracles.h"
TEST_CASE("Euler gas one-step flux agrees with independent unsplit face oracle", "[euler]") {
    for (const auto dims : {std::pair<std::size_t, std::size_t>{7, 5}, {2, 5}, {5, 2}, {2, 2}}) {
        EulerGasGridConfig g{dims.first, dims.second, .7, 1.3, 1.4};
        PeriodicEulerGasGrid grid(g);
        auto s = grid.state();
        for (std::size_t k = 0; k < s.density.size(); ++k)
            Put(s, k,
                Conserved(.9 + .15 * std::sin(double(k)), .3 * std::cos(double(k)),
                          -.2 * std::sin(2.0 * k), 1 + .1 * std::cos(3.0 * k)));
        grid.setState(s);
        const auto expected = Oracle(s, g, .001);
        const auto d = grid.step(.001);
        const auto actual = grid.state();
        REQUIRE(d.substeps == 1);
        Audit(s, actual, d, g.spacingX * g.spacingY);
        for (std::size_t k = 0; k < s.density.size(); ++k)
            for (std::size_t a = 0; a < 4; ++a)
                Near(At(actual, k)[a], At(expected, k)[a], 2e-15);
    }
}
TEST_CASE("Euler gas uniform mode, owned snapshots and replay are exact", "[euler]") {
    EulerGasGridConfig g{4, 3, .5, .7, 1.4};
    PeriodicEulerGasGrid a(g), b(g);
    auto s = a.state();
    for (std::size_t k = 0; k < s.density.size(); ++k)
        Put(s, k, Conserved(2, -.7, .2, 3));
    a.setState(s);
    b.setState(s);
    auto copy = s;
    s.density[0] = 999;
    auto cfg = a.config();
    cfg.gamma = 20;
    auto primitives = a.primitives();
    primitives.pressure[0] = 999;
    EulerGasStepConfig options;
    options.maxSubstep = .01;
    options.maximumSubsteps = 1;
    options.maximumCellVisits = 10 * copy.density.size();
    const auto d = a.step(.01, options);
    b.step(.01, options);
    Same(a.state(), copy);
    Same(a.state(), b.state());
    REQUIRE(Values(d) == Values(b.lastStep()));
    REQUIRE(d.substeps == 1);
    Audit(copy, a.state(), d, g.spacingX * g.spacingY);
    auto retained = a.state();
    auto retainedD = a.lastStep();
    a.setState(copy);
    a.step(.02);
    Same(retained, copy);
    REQUIRE(Values(retainedD) == Values(d));
    REQUIRE(a.time() == .03);
    const auto before = a.state();
    const auto zero = a.step(0, EulerGasStepConfig{.9, .1, 0, 3 * copy.density.size()});
    Same(before, a.state());
    REQUIRE(zero.zeroDurationNoOp);
    REQUIRE(zero.cellVisits == 3 * copy.density.size());
    REQUIRE(zero.substeps == 0);
    REQUIRE(zero.lastSubstep == 0);
    REQUIRE(zero.timeBefore == zero.timeAfter);
}
TEST_CASE("Euler gas periodic moving contact converges with measured first-order diffusion",
          "[euler][continuum]") {
    const auto a = Contact(32), b = Contact(64), c = Contact(128), d = Contact(256);
    std::cout << "Euler contact RMS: " << a.error << ", " << b.error << ", " << c.error << ", "
              << d.error << '\n';
    REQUIRE(b.error < .7 * a.error);
    REQUIRE(c.error < .65 * b.error);
    REQUIRE(b.error / c.error > 1.65);
    REQUIRE(d.error < .6 * c.error);
    REQUIRE(c.error / d.error > 1.8);
    REQUIRE(a.amplitude < b.amplitude);
    REQUIRE(b.amplitude < c.amplitude);
    REQUIRE(c.amplitude < .2);
    REQUIRE(c.amplitude < d.amplitude);
    REQUIRE(d.amplitude < .2);
    for (const auto r : {a, b, c, d}) {
        REQUIRE(r.pressureError < 2e-13);
        REQUIRE(r.velocityError < 2e-13);
    }
}
TEST_CASE("Euler gas smooth nonlinear acoustic simple wave converges before breaking",
          "[euler][continuum]") {
    const double a = SimpleWaveError(32), b = SimpleWaveError(64), c = SimpleWaveError(128),
                 d = SimpleWaveError(256);
    std::cout << "Euler nonlinear simple-wave L1: " << a << ", " << b << ", " << c << ", " << d
              << '\n';
    REQUIRE(b < .7 * a);
    REQUIRE(c < .65 * b);
    REQUIRE(d < .6 * c);
    REQUIRE(c / d > 1.7);
}
TEST_CASE(
    "Euler gas shock tube matches independent exact Riemann solution and isolated-image control",
    "[euler][continuum]") {
    const Sod sod;
    Near(sod.pStar, .303130178050647, 2e-15);
    Near(sod.uStar, .92745262004895, 3e-15);
    const auto a = Shock(128, 2), b = Shock(256, 2), c = Shock(512, 2);
    std::cout << "Euler Sod conserved L1 window: " << a.error << ", " << b.error << ", " << c.error
              << '\n';
    REQUIRE(b.error < .8 * a.error);
    REQUIRE(c.error < .8 * b.error);
    REQUIRE(c.error < .08);
    // Same dx and central states, twice the separation from periodic images.
    // Global speeds can differ slightly as distant waves evolve. Compare stored
    // conservative fields in the oracle window, relative to discretization error.
    const auto control = Shock(512, 4);
    double imageDifference = 0;
    for (std::size_t i = 0; i < 256; ++i)
        if (std::abs((i + .5) * 2 / 256 - 1) < .4)
            for (std::size_t q = 0; q < 4; ++q)
                imageDifference +=
                    2.0 / 256 * std::abs(At(b.state, i)[q] - At(control.state, i + 128)[q]);
    std::cout << "Euler doubled-domain image difference: " << imageDifference << '\n';
    REQUIRE(imageDifference < 1e-5 * b.error);
}
TEST_CASE("Euler gas positivity split proof supports gamma above two and near one",
          "[euler][positivity]") {
    for (double gamma : {std::nextafter(1.0, 2.0), 1.01, 1.4, 3.0, 20.0, 1e150}) {
        EulerGasGridConfig g{4, 2, .3, .7, gamma};
        PeriodicEulerGasGrid grid(g);
        auto s = grid.state();
        for (std::size_t k = 0; k < s.density.size(); ++k) {
            const double rho = 1 + .1 * double(k),
                         pressure = gamma > 1e100 ? 1e-150 : .3 + .01 * double(k);
            // Very large gamma requires small velocities so positive internal
            // energy survives the unavoidable stored E-K subtraction.
            const double speed = gamma > 1e100 ? 1e-150 : .02;
            Put(s, k,
                Conserved(rho, speed * std::sin(double(k)), speed * std::cos(double(k)), pressure,
                          gamma));
        }
        grid.setState(s);
        const auto before = grid.state();
        const auto d = grid.step(.001);
        Audit(before, grid.state(), d, .21);
        const auto q = grid.primitives();
        for (std::size_t k = 0; k < s.density.size(); ++k) {
            REQUIRE(q.pressure[k] > 0);
            REQUIRE(q.internalEnergy[k] > 0);
        }
    }
    // Check the LF split algebra independently on ordinary states, including
    // gamma > 2. Neither a production limiter nor an arbitrary gamma ceiling.
    for (double gamma : {1.01, 1.4, 3.0, 20.0})
        for (double u : {-20., -.1, 0., .1, 20.}) {
            const double rho = .7, pressure = 2,
                         alpha = std::abs(u) + std::sqrt(gamma * pressure / rho);
            const auto q = Conserved(rho, u, -.3, pressure, gamma), f = Flux(q, true, gamma);
            for (double sign : {-1., 1.}) {
                Cell split{};
                for (std::size_t a = 0; a < 4; ++a)
                    split[a] = q[a] + sign * f[a] / alpha;
                REQUIRE(split[0] > 0);
                REQUIRE(split[3] - .5 * (split[1] * split[1] + split[2] * split[2]) / split[0] > 0);
            }
        }
}
TEST_CASE("Euler gas finite scales, cold subnormal mode and clock range", "[euler][range]") {
    for (double scale : {1e-150, 1e150, 1e300}) {
        EulerGasGridConfig g{3, 2, .1, .1, 1.4};
        PeriodicEulerGasGrid grid(g);
        auto s = grid.state();
        for (std::size_t k = 0; k < s.density.size(); ++k)
            Put(s, k, Conserved(scale, .1, -.2, scale));
        grid.setState(s);
        const auto d = grid.step(.001);
        Same(grid.state(), s);
        REQUIRE(d.final.minimumPressure > 0);
        Near(d.totalEnergyDefect, 0, 0);
        REQUIRE(d.totalEnergyRoundoffAllowance < d.initial.totalEnergy * 1e-10);
    }
    EulerGasGridConfig g{2, 2, 1e154, 1e154, 1.4};
    PeriodicEulerGasGrid cold(g);
    auto s = cold.state();
    std::fill(s.density.begin(), s.density.end(), .1);
    std::fill(s.totalEnergy.begin(), s.totalEnergy.end(), 1e-320);
    cold.setState(s);
    EulerGasStepConfig options;
    options.maxSubstep = 1e308;
    options.maximumSubsteps = 1;
    const auto d = cold.step(1e308, options);
    Same(cold.state(), s);
    REQUIRE(d.substeps == 1);
    REQUIRE(cold.primitives().pressure[0] > 0);
    REQUIRE(d.final.totalEnergy > 0);
    REQUIRE(cold.time() == 1e308);
    for (double duration : {1., 1e308}) {
        REQUIRE_THROWS(cold.step(duration, options));
        Same(cold.state(), s);
        REQUIRE(Values(cold.lastStep()) == Values(d));
        REQUIRE(cold.time() == 1e308);
    }
}
TEST_CASE("Euler gas cold nonuniform compressions remain strictly admissible without floors",
          "[euler][positivity]") {
    for (double scale : {1e-150, 1., 1e150}) {
        EulerGasGridConfig g{12, 8, .1, .17, 1.4};
        PeriodicEulerGasGrid grid(g);
        auto s = grid.state();
        for (std::size_t j = 0; j < g.rows; ++j)
            for (std::size_t i = 0; i < g.columns; ++i) {
                const double x = 2 * Pi * (i + .5) / g.columns;
                const double y = 2 * Pi * (j + .5) / g.rows;
                Put(s, i + g.columns * j,
                    Conserved(scale * (1 + .4 * std::cos(x + y)), .3 * std::sin(x),
                              -.2 * std::cos(y), 1e-12 * scale));
            }
        grid.setState(s);
        const auto d = grid.step(.2);
        const auto actual = grid.state();
        const auto q = grid.primitives();
        REQUIRE(d.substeps > 1);
        for (std::size_t k = 0; k < s.density.size(); ++k) {
            REQUIRE(actual.density[k] > 0);
            REQUIRE(q.pressure[k] > 0);
            REQUIRE(q.internalEnergy[k] > 0);
        }
        REQUIRE(std::abs(d.massDefect) <= d.massRoundoffAllowance);
        REQUIRE(std::abs(d.totalEnergyDefect) <= d.totalEnergyRoundoffAllowance);
        REQUIRE(d.maximumCfl <= .9);
    }
}
TEST_CASE("Euler gas rejects shape, admissibility and geometry errors transactionally",
          "[euler][validation]") {
    for (const auto g : {EulerGasGridConfig{1, 2, 1, 1, 1.4},
                         {2, 1, 1, 1, 1.4},
                         {EulerGasGridConfig::MaximumCells, 2, 1, 1, 1.4},
                         {std::numeric_limits<std::size_t>::max(), 2, 1, 1, 1.4},
                         {2, 2, 0, 1, 1.4},
                         {2, 2, 1, -1, 1.4},
                         {2, 2, 1e308, 1, 1.4},
                         {2, 2, 1e-300, 1e-300, 1.4},
                         {2, 2, 1, 1, 1},
                         {2, 2, 1, 1, std::numeric_limits<double>::infinity()},
                         {2, 2, std::numeric_limits<double>::quiet_NaN(), 1, 1.4},
                         {2, 2, 1, 1, std::numeric_limits<double>::quiet_NaN()}})
        REQUIRE_THROWS(PeriodicEulerGasGrid(g));
    PeriodicEulerGasGrid grid({4, 2, .3, .7, 1.4});
    grid.step(.001);
    const auto before = grid.state();
    const auto d = grid.lastStep();
    for (int variant = 0; variant < 10; ++variant) {
        auto s = before;
        if (variant == 0)
            s.density.pop_back();
        if (variant == 1)
            s.momentumX.pop_back();
        if (variant == 2)
            s.momentumY.pop_back();
        if (variant == 3)
            s.totalEnergy.pop_back();
        if (variant == 4)
            s.density[0] = 0;
        if (variant == 5)
            s.totalEnergy[0] = 0;
        if (variant == 6)
            s.momentumX[0] = std::numeric_limits<double>::quiet_NaN();
        if (variant == 7)
            s.momentumY[0] = std::numeric_limits<double>::infinity();
        if (variant == 8) {
            s.momentumX[0] = 2;
            s.totalEnergy[0] = 2;
        } // Rounded E-K=0.
        if (variant == 9) {
            s.density[0] = 1e-320;
            s.totalEnergy[0] = 1e308;
        } // Unrepresentable sound speed.
        REQUIRE_THROWS(grid.setState(s));
        Same(grid.state(), before);
        REQUIRE(Values(grid.lastStep()) == Values(d));
        REQUIRE(grid.time() == .001);
    }
}
TEST_CASE("Euler gas budget, arithmetic and late staged failures retain the complete snapshot",
          "[euler][rollback]") {
    PeriodicEulerGasGrid grid({4, 2, .1, .2, 1.4});
    auto start = grid.state();
    for (std::size_t k = 0; k < start.density.size(); ++k)
        Put(start, k, Conserved(1 + .1 * std::sin(double(k)), .2, -.1, 1));
    grid.setState(start);
    grid.step(.001);
    const auto before = grid.state();
    const auto prior = grid.lastStep();
    auto retains = [&] {
        Same(grid.state(), before);
        REQUIRE(Values(grid.lastStep()) == Values(prior));
        REQUIRE(grid.time() == .001);
    };
    for (double duration :
         {-1., std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        REQUIRE_THROWS(grid.step(duration));
        retains();
    }
    for (const auto options : {EulerGasStepConfig{0, .1, 10, 1000},
                               {1, .1, 10, 1000},
                               {.9, 0, 10, 1000},
                               {.9, .1, 0, 1000},
                               {.9, .1, 1, 1000},
                               {.9, .1, 100, 79},
                               {.9, .001, 100, 80},
                               {.9, .1, EulerGasStepConfig::MaximumSubsteps + 1, 1000},
                               {.9, .1, 10, EulerGasStepConfig::MaximumCellVisits + 1}}) {
        REQUIRE_THROWS(grid.step(.2, options));
        retains();
    }
    REQUIRE_THROWS(grid.step(0, EulerGasStepConfig{.9, .1, 0, 23}));
    retains();
    // Counts right at the hard cell cap remain valid before allocation.
    EulerGasGridConfig{512, 512, 1, 1, 1.4}.Validate();
    // All supplied fields are individually finite/admissible. Compression makes
    // the next density exceed float64 only during the staged face update.
    PeriodicEulerGasGrid large({4, 2, 1e-154, 1e-154, 1.4});
    auto s = large.state();
    for (std::size_t k = 0; k < s.density.size(); ++k) {
        s.density[k] = 1.5e308;
        s.totalEnergy[k] = 1.7e308;
        s.momentumX[k] = (k % 4 < 2 ? 1 : -1) * .75e308;
    }
    large.setState(s);
    const auto d = large.step(0);
    REQUIRE_THROWS_AS(large.step(4e-155), std::overflow_error);
    Same(large.state(), s);
    REQUIRE(Values(large.lastStep()) == Values(d));
    REQUIRE(large.time() == 0);
    // Rate overflow is checked on step, including zero-duration observations.
    PeriodicEulerGasGrid tiny({2, 2, 1e-308, 1, 1.4});
    auto q = tiny.state();
    for (std::size_t k = 0; k < q.density.size(); ++k)
        Put(q, k, Conserved(1, 100, 0, 1));
    tiny.setState(q);
    REQUIRE_THROWS(tiny.step(0));
    Same(tiny.state(), q);
    REQUIRE(tiny.lastStep().cellVisits == 0);
}
TEST_CASE("Euler gas strict rounded CFL and representable scaled fluxes", "[euler][range]") {
    PeriodicEulerGasGrid grid({2, 2, .1, .2, 1.4});
    EulerGasStepConfig options;
    options.maximumSubsteps = 1;
    // The independent real-valued CFL endpoint exceeds the rounded-down native
    // limit by a few ulps. A one-substep budget must not silently admit it.
    const double endpoint = options.cflSafety / (std::sqrt(1.4) * (10 + 5));
    REQUIRE_THROWS(grid.step(endpoint, options));
    REQUIRE(grid.time() == 0);
    double inside = endpoint;
    for (int i = 0; i < 20; ++i)
        inside = std::nextafter(inside, 0.0);
    const auto d = grid.step(inside, options);
    REQUIRE(d.substeps == 1);
    REQUIRE(d.maximumCfl <= options.cflSafety);
    // Raw E+p and (E+p)*u exceed float64. The complete scaled transfer and
    // stored uniform state are representable, so neither intermediate is formed.
    PeriodicEulerGasGrid huge({3, 2, .1, .1, 1.4});
    auto s = huge.state();
    for (std::size_t k = 0; k < s.density.size(); ++k) {
        s.density[k] = 1e307;
        s.momentumX[k] = 2e307;
        s.totalEnergy[k] = 1.7e308;
    }
    huge.setState(s);
    const auto hd = huge.step(.001);
    Same(huge.state(), s);
    REQUIRE(hd.totalEnergyDefect == 0);
    REQUIRE(hd.final.minimumPressure > 0);
}
