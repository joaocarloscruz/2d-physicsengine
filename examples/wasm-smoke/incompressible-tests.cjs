const assert = require('node:assert/strict');
const pi = Math.PI;
const defaults = {
    cflSafety : .8,
    absoluteDivergenceTolerance : 1e-10,
    relativeDivergenceTolerance : 1e-10,
    absoluteVelocityTolerance : 1e-10,
    relativeVelocityTolerance : 1e-10,
    maximumSubsteps : 1000,
    maximumIterations : 10000,
    maximumCellVisits : 100000000
};
const config = (nx = 8, ny = 6, extra = {}) => ({
    geometry : {columns : nx, rows : ny, spacingX : 2 * pi / nx, spacingY : 2 * pi / ny},
    density : 1,
    kinematicViscosity : 0,
    ...extra
});
const close = (a, b, t, label = '') =>
    assert.ok(Math.abs(a - b) <= t, `${label}: ${a} != ${b}; margin ${t}`);
function snapshot(g) {
    return {
        config : g.config(),
        velocities : g.velocities(),
        lastStep : g.lastStep(),
        lastProjection : g.lastProjection(),
        lastDiffusion : g.lastDiffusion(),
        time : g.time()
    };
}
function taylorGreen(c, amplitude = 1) {
    const g = c.geometry, n = g.columns * g.rows, x = Array(n), y = Array(n);
    for (let j = 0; j < g.rows; ++j)
        for (let i = 0; i < g.columns; ++i) {
            x[i + g.columns * j] = amplitude * Math.sin(i * g.spacingX) * Math.cos((j + .5) * g.spacingY);
            y[i + g.columns * j] = -amplitude * Math.cos((i + .5) * g.spacingX) * Math.sin(j * g.spacingY);
        }
    return {xFaces : x, yFaces : y};
}
function rms(a, b) {
    let e = 0;
    for (let k = 0; k < a.xFaces.length; ++k)
        e += (a.xFaces[k] - b.xFaces[k]) ** 2 + (a.yFaces[k] - b.yFaces[k]) ** 2;
    return Math.sqrt(e / a.xFaces.length);
}
const id = (i, j, g) => ((i % g.columns) + g.columns) % g.columns + g.columns * (((j % g.rows) + g.rows) % g.rows);

// ----- Independent dense oracles: no production transfers or iterative solves -----
function denseSolve(a, b) {
    a = a.map(row => row.slice()); b = b.slice();
    const n = b.length;
    for (let i = 0; i < n; ++i) {
        let pivot = i;
        for (let j = i + 1; j < n; ++j) if (Math.abs(a[j][i]) > Math.abs(a[pivot][i])) pivot = j;
        [a[i], a[pivot]] = [a[pivot], a[i]]; [b[i], b[pivot]] = [b[pivot], b[i]];
        assert.ok(Math.abs(a[i][i]) > 1e-14);
        const diagonal = a[i][i];
        for (let k = i; k < n; ++k) a[i][k] /= diagonal;
        b[i] /= diagonal;
        for (let j = 0; j < n; ++j)
            if (j !== i) {
                const factor = a[j][i];
                for (let k = i; k < n; ++k) a[j][k] -= factor * a[i][k];
                b[j] -= factor * b[i];
            }
    }
    return b;
}
function denseOperator(g, identity, scale) {
    const n = g.columns * g.rows, a = Array.from({length : n}, () => Array(n).fill(0));
    for (let j = 0; j < g.rows; ++j)
        for (let i = 0; i < g.columns; ++i) {
            const k = id(i, j, g);
            a[k][k] += identity;
            for (const [di, dj] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
                const weight = scale / (di ? g.spacingX : g.spacingY) ** 2;
                a[k][k] += weight; a[k][id(i + di, j + dj, g)] -= weight;
            }
        }
    return a;
}
// Discrete projection; the gauge replaces the last Poisson equation.
function denseProject(q, g) {
    const n = q.xFaces.length, rhs = Array(n);
    for (let j = 0; j < g.rows; ++j)
        for (let i = 0; i < g.columns; ++i) {
            const k = id(i, j, g);
            rhs[k] = -(q.xFaces[id(i + 1, j, g)] - q.xFaces[k]) / g.spacingX
                - (q.yFaces[id(i, j + 1, g)] - q.yFaces[k]) / g.spacingY;
        }
    const a = denseOperator(g, 0, 1);
    a[n - 1] = Array(n).fill(1); rhs[n - 1] = 0;
    const potential = denseSolve(a, rhs);
    for (let j = 0; j < g.rows; ++j)
        for (let i = 0; i < g.columns; ++i) {
            const k = id(i, j, g);
            q.xFaces[k] -= (potential[k] - potential[id(i - 1, j, g)]) / g.spacingX;
            q.yFaces[k] -= (potential[k] - potential[id(i, j - 1, g)]) / g.spacingY;
        }
    return potential;
}
// Oriented donor fluxes through both faces of each staggered dual cell; on
// two-cell axes the two faces are distinct edges between the same samples.
function independentFlux(q, g, h) {
    const next = {xFaces : q.xFaces.slice(), yFaces : q.yFaces.slice()};
    for (const component of [0, 1]) {
        const field = component ? q.yFaces : q.xFaces, out = component ? next.yFaces : next.xFaces;
        for (let j = 0; j < g.rows; ++j)
            for (let i = 0; i < g.columns; ++i) {
                let change = 0;
                for (const axis of [0, 1])
                    for (const side of [-1, 1]) {
                        const x = i - (axis === 0 && side === -1), y = j - (axis === 1 && side === -1);
                        const lo = id(x, y, g), hi = id(x + (axis === 0), y + (axis === 1), g);
                        let speed;
                        if (component === 0 && axis === 0) speed = (q.xFaces[lo] + q.xFaces[hi]) / 2;
                        else if (component === 0) speed = (q.yFaces[id(x - 1, y + 1, g)] + q.yFaces[id(x, y + 1, g)]) / 2;
                        else if (axis === 0) speed = (q.xFaces[id(x + 1, y - 1, g)] + q.xFaces[id(x + 1, y, g)]) / 2;
                        else speed = (q.yFaces[lo] + q.yFaces[hi]) / 2;
                        change -= side * h * speed * field[speed >= 0 ? lo : hi] / (axis === 0 ? g.spacingX : g.spacingY);
                    }
                out[id(i, j, g)] += change;
            }
    }
    return next;
}
// Complex helpers for exact transported-shear multipliers.
const cmul = (a, b) => [a[0] * b[0] - a[1] * b[1], a[0] * b[1] + a[1] * b[0]];
const cdiv = (a, b) => { const d = b[0] * b[0] + b[1] * b[1]; return [(a[0] * b[0] + a[1] * b[1]) / d, (a[1] * b[0] - a[0] * b[1]) / d]; };
const cexp = a => [Math.exp(a[0]) * Math.cos(a[1]), Math.exp(a[0]) * Math.sin(a[1])];

function interfaceAndOwnership(p) {
    const fresh = new p.PeriodicIncompressibleGrid();
    assert.deepEqual(fresh.config(), {geometry : {columns : 16, rows : 16, spacingX : 1, spacingY : 1},
                                      density : 1, kinematicViscosity : 0});
    assert.equal(fresh.time(), 0);
    assert.equal(fresh.velocities().xFaces.length, 256);
    assert.deepEqual(fresh.lastStep().history, []);
    fresh.delete();

    const c = config(2, 3, {kinematicViscosity : .3}), g = new p.PeriodicIncompressibleGrid(c);
    const q = {xFaces : Array(6).fill(.7), yFaces : Array(6).fill(-.2)};
    g.setVelocities(q.xFaces, q.yFaces);
    const d = g.step(.3);
    assert.deepEqual(g.velocities(), q);
    assert.equal(g.time(), .3); assert.equal(d.iterations, 0);
    assert.equal(d.initialKineticEnergy, d.finalKineticEnergy);
    assert.equal(d.storageEnergyError, 0); assert.equal(d.donorDissipation, 0);
    assert.equal(d.history.length, d.substeps);
    assert.deepEqual(g.lastStep(), d);
    // Every returned value is an owning copy, including nested history.
    d.history[0].timeStep = 99; d.history[0].projection.iterations = 99;
    assert.notEqual(g.lastStep().history[0].timeStep, 99);
    assert.notEqual(g.lastStep().history[0].projection.iterations, 99);
    const projection = g.lastProjection(), diffusion = g.lastDiffusion();
    const copy = g.lastProjection(); copy.pressure[0] = 7; copy.potential[0] = 7;
    assert.deepEqual(g.lastProjection(), projection);
    const velocities = g.velocities(); velocities.xFaces[0] = 9;
    assert.equal(g.velocities().xFaces[0], .7);
    // Zero duration publishes a fresh zero-step report and preserves operators.
    const z = g.step(0);
    assert.equal(z.zeroStepNoOp, true); assert.equal(z.substeps, 0); assert.equal(z.cellVisits, 18);
    assert.equal(z.initialTime, .3); assert.equal(z.finalTime, .3); assert.equal(g.time(), .3);
    assert.deepEqual(g.lastProjection(), projection); assert.deepEqual(g.lastDiffusion(), diffusion);
    // Setters retain the clock and every historical observer.
    g.setVelocities(q.xFaces, q.yFaces);
    assert.equal(g.time(), .3); assert.equal(g.lastStep().zeroStepNoOp, true);
    assert.deepEqual(g.lastProjection(), projection);
    g.delete();
}

function denseCoupledOracle(p) {
    for (const [nx, ny] of [[2, 3], [3, 2], [4, 3]]) {
        const c = config(nx, ny, {density : 1.7, kinematicViscosity : .23});
        c.geometry.spacingX = .7; c.geometry.spacingY = 1.3;
        const g = new p.PeriodicIncompressibleGrid(c), n = nx * ny;
        const q = {xFaces : [], yFaces : []};
        for (let k = 0; k < n; ++k) { q.xFaces.push(.4 + Math.sin(1.3 * k)); q.yFaces.push(-.2 + Math.cos(.7 * k)); }
        g.setVelocities(q.xFaces, q.yFaces);
        const expected = {xFaces : q.xFaces.slice(), yFaces : q.yFaces.slice()};
        denseProject(expected, c.geometry);
        let next = independentFlux(expected, c.geometry, .03);
        const diffusion = denseOperator(c.geometry, 1, .03 * c.kinematicViscosity);
        next = {xFaces : denseSolve(diffusion, next.xFaces), yFaces : denseSolve(diffusion, next.yFaces)};
        const potential = denseProject(next, c.geometry);
        const d = g.step(.03);
        assert.equal(d.substeps, 1);
        assert.ok(d.initialProjection.initialDivergenceRms > 0, 'input starts compressible');
        assert.ok(rms(g.velocities(), next) < 2e-10, `${nx}x${ny} velocity ${rms(g.velocities(), next)}`);
        const pressure = g.lastProjection().pressure;
        for (let k = 0; k < n; ++k) close(pressure[k], c.density * potential[k] / .03, 3e-8, 'pressure');
        // Work counts: native operator visits plus 14 + 17 * substeps full scans.
        let native = d.initialProjection.cellVisits;
        for (const s of d.history) native += s.diffusion.cellVisits + s.projection.cellVisits;
        assert.equal(d.cellVisits, native + (14 + 17 * d.substeps) * n);
        assert.ok(Math.abs(d.storageEnergyError) <= d.roundoffEnergyAllowance);
        const replay = new p.PeriodicIncompressibleGrid(c);
        replay.setVelocities(q.xFaces, q.yFaces);
        const again = replay.step(.03);
        assert.deepEqual(replay.velocities(), g.velocities());
        assert.deepEqual(again, d);
        assert.deepEqual(replay.lastProjection().pressure, pressure);
        replay.delete(); g.delete();
    }
}

function transportedShear(p) {
    for (const mean of [-.7, 0, .7]) {
        const c = config(12, 2, {kinematicViscosity : .13}), g = new p.PeriodicIncompressibleGrid(c);
        const theta = c.geometry.spacingX, q = {xFaces : [], yFaces : []};
        for (let j = 0; j < 2; ++j)
            for (let i = 0; i < 12; ++i) { q.xFaces.push(mean); q.yFaces.push(Math.sin((i + .5) * theta)); }
        g.setVelocities(q.xFaces, q.yFaces);
        const d = g.step(2.1);
        const lambda = 4 * (Math.sin(theta / 2) / c.geometry.spacingX) ** 2;
        let factor = [1, 0];
        for (const s of d.history) {
            const shift = cexp([0, mean >= 0 ? -theta : theta]);
            const donor = [1 - Math.abs(mean) * s.timeStep / c.geometry.spacingX * (1 - shift[0]),
                           Math.abs(mean) * s.timeStep / c.geometry.spacingX * shift[1]];
            factor = cdiv(cmul(factor, donor), [1 + c.kinematicViscosity * s.timeStep * lambda, 0]);
            assert.ok(s.outgoingCfl <= .8 + 1e-15);
            assert.ok(s.maximumDualDivergence < 1e-12);
            assert.ok(Math.abs(s.advectionStorageError) <= s.roundoffEnergyAllowance);
        }
        const expected = {xFaces : q.xFaces, yFaces : []};
        for (let j = 0; j < 2; ++j)
            for (let i = 0; i < 12; ++i) expected.yFaces.push(cmul(factor, cexp([0, (i + .5) * theta]))[1]);
        assert.ok(rms(g.velocities(), expected) < 2e-10, `shear ${mean}: ${rms(g.velocities(), expected)}`);
        assert.ok(Math.abs(d.finalMeanX - d.initialMeanX) <= d.meanRoundoffAllowanceX);
        assert.ok(Math.abs(d.storageEnergyError) <= d.roundoffEnergyAllowance);
        assert.ok(d.donorDissipation >= 0); assert.ok(d.viscousDissipation >= 0);
        assert.ok(Math.abs(d.projectionResidualWork) < 1e-16);
        g.delete();
    }
}

// Spatial: errors against the continuum Taylor-Green vortex at matched sample
// locations (faces for velocity, cell centers for pressure). The pressure is a
// point sample, not a cell average, so midpoint quadrature is not implied.
function taylorGreenSpatial(p) {
    const tight = {...defaults, absoluteDivergenceTolerance : 1e-12, relativeDivergenceTolerance : 0};
    let previous = [1, 1, 1];
    const reported = [];
    for (const nx of [12, 24, 48]) {
        const c = config(nx, nx, {density : 1.3, kinematicViscosity : .1}), g = new p.PeriodicIncompressibleGrid(c);
        const start = taylorGreen(c);
        g.setVelocities(start.xFaces, start.yFaces);
        for (let k = 0; k < 20; ++k) g.step(.001, tight);
        const t = .02, nu = c.kinematicViscosity;
        const velocity = rms(g.velocities(), taylorGreen(c, Math.exp(-2 * nu * t)));
        const pressure = g.lastProjection().pressure;
        let pressureError = 0;
        for (let j = 0; j < nx; ++j)
            for (let i = 0; i < nx; ++i) {
                const exact = c.density / 4 * Math.exp(-4 * nu * t)
                    * (Math.cos(2 * (i + .5) * c.geometry.spacingX) + Math.cos(2 * (j + .5) * c.geometry.spacingY));
                pressureError += (pressure[i + nx * j] - exact) ** 2;
            }
        pressureError = Math.sqrt(pressureError / (nx * nx));
        const energy = Math.abs(g.lastStep().finalKineticEnergy - c.density * pi * pi * Math.exp(-4 * nu * t));
        const errors = [velocity, pressureError, energy];
        errors.forEach((e, k) => assert.ok(e < previous[k] * .75, `spatial ${nx}: ${errors} after ${previous}`));
        previous = errors; reported.push(errors);
        g.delete();
    }
    // Deliberately coarse inviscid donor transport loses energy; report it.
    const c = config(4, 4), coarse = new p.PeriodicIncompressibleGrid(c), tg = taylorGreen(c);
    coarse.setVelocities(tg.xFaces, tg.yFaces);
    const d = coarse.step(.4);
    assert.ok(d.donorDissipation > 0); assert.equal(d.viscousDissipation, 0);
    assert.ok(d.finalKineticEnergy < d.initialKineticEnergy);
    coarse.delete();
    return reported;
}

// Temporal: fixed 12x12 grid against a 256-step numerical reference, so the
// spatial error cancels and only first-order splitting error remains.
function taylorGreenTemporal(p) {
    const c = config(12, 12, {kinematicViscosity : .1});
    const tight = {...defaults, absoluteDivergenceTolerance : 1e-13, relativeDivergenceTolerance : 0};
    const run = steps => {
        const g = new p.PeriodicIncompressibleGrid(c), tg = taylorGreen(c);
        g.setVelocities(tg.xFaces, tg.yFaces);
        for (let k = 0; k < steps; ++k) g.step(.04 / steps, tight);
        const result = [g.velocities(), g.lastProjection().pressure];
        g.delete();
        return result;
    };
    const reference = run(256);
    let previous = [1, 1];
    const reported = [];
    for (const steps of [4, 8, 16, 32]) {
        const [velocities, pressure] = run(steps);
        let pressureError = 0;
        for (let k = 0; k < pressure.length; ++k) pressureError += (pressure[k] - reference[1][k]) ** 2;
        const errors = [rms(velocities, reference[0]), Math.sqrt(pressureError / pressure.length)];
        errors.forEach((e, k) => assert.ok(e < previous[k] * .65, `temporal ${steps}: ${errors} after ${previous}`));
        previous = errors; reported.push(errors);
    }
    return reported;
}

function ledgersAndResidual(p) {
    const c = config(4, 3);
    c.geometry.spacingX = .7; c.geometry.spacingY = 1.3;
    const g = new p.PeriodicIncompressibleGrid(c), q = {xFaces : [], yFaces : []};
    for (let k = 0; k < 12; ++k) { q.xFaces.push(.7 + .2 * Math.sin(k)); q.yFaces.push(.3 * Math.cos(.7 * k)); }
    g.setVelocities(q.xFaces, q.yFaces);
    const d = g.step(.02, {...defaults, absoluteDivergenceTolerance : 1, relativeDivergenceTolerance : 0});
    assert.ok(d.initialProjection.finalDivergenceRms > 1e-3);
    assert.ok(d.history[0].maximumDualDivergence > 1e-3); assert.ok(d.history[0].maximumRowSum > 1);
    assert.ok(Math.abs(d.dualDivergenceWork) > 1e-6);
    assert.ok(Math.abs(d.storageEnergyError) <= d.roundoffEnergyAllowance);
    assert.ok(d.history[0].advectionEnergyChange <= d.history[0].advectionEnergyBound + d.history[0].roundoffEnergyAllowance);
    // The step ledger is the sum of its substep and operator records.
    let donor = 0, viscous = 0;
    for (const s of d.history) { donor += s.donorDissipation; viscous += s.diffusion.gradientDissipation; }
    close(d.donorDissipation, donor, 1e-15 * Math.max(1, donor), 'donor ledger');
    close(d.viscousDissipation, viscous, 1e-15 * Math.max(1, viscous), 'viscous ledger');
    g.delete();
    let reference = 0;
    for (const rho of [1e-6, 1, 1e6]) {
        const scaled = config(8, 6, {density : rho, kinematicViscosity : .2});
        const s = new p.PeriodicIncompressibleGrid(scaled), tg = taylorGreen(scaled);
        s.setVelocities(tg.xFaces, tg.yFaces);
        const r = s.step(.02), normalized = r.finalKineticEnergy / rho;
        if (reference === 0) reference = normalized;
        else close(normalized, reference, 1e-12 * reference, 'density scaling');
        assert.ok(Math.abs(r.storageEnergyError) <= r.roundoffEnergyAllowance);
        s.delete();
    }
}

function budgetsAndRollback(p) {
    const c = config(8, 6, {kinematicViscosity : .2}), g = new p.PeriodicIncompressibleGrid(c), tg = taylorGreen(c);
    g.setVelocities(tg.xFaces, tg.yFaces);
    g.step(.02);
    const saved = snapshot(g), reference = new p.PeriodicIncompressibleGrid(c);
    reference.setVelocities(saved.velocities.xFaces, saved.velocities.yFaces);
    const successful = reference.step(.02), n = 48;
    for (const options of [{maximumCellVisits : 1}, {maximumCellVisits : successful.cellVisits - n},
                           {maximumIterations : 0}, {maximumSubsteps : 0},
                           {maximumSubsteps : 1, cflSafety : .001}]) {
        assert.throws(() => g.step(.02, {...defaults, ...options}), e => e instanceof Error && !('excPtr' in e));
        assert.deepEqual(snapshot(g), saved);
    }
    const exact = g.step(.02, {...defaults, maximumIterations : successful.iterations,
                                maximumCellVisits : successful.cellVisits});
    assert.equal(exact.cellVisits, successful.cellVisits);
    assert.deepEqual(g.velocities(), reference.velocities());
    const bounded = new p.PeriodicIncompressibleGrid(c);
    bounded.setVelocities(saved.velocities.xFaces, saved.velocities.yFaces);
    const before = snapshot(bounded);
    assert.throws(() => bounded.step(.02, {...defaults, maximumIterations : successful.iterations - 1}));
    assert.deepEqual(snapshot(bounded), before);
    // A late final stored-state audit failure also rolls back completely.
    const small = new p.PeriodicIncompressibleGrid(config(2, 2)), difficult = {xFaces : Array(4).fill(1e12), yFaces : Array(4).fill(-1e12)};
    difficult.xFaces[0] = 1e12 + 2 ** -13; // Next representable float64 above 1e12.
    small.setVelocities(difficult.xFaces, difficult.yFaces);
    const smallBefore = snapshot(small);
    assert.throws(() => small.step(.01, {...defaults, absoluteDivergenceTolerance : 1e-6,
                                         relativeDivergenceTolerance : 0, maximumSubsteps : 1}));
    assert.deepEqual(snapshot(small), smallBefore);
    for (const grid of [g, reference, bounded, small]) grid.delete();
}

function invalidControls(p) {
    for (const bad of [-1, Infinity, NaN]) {
        const g = new p.PeriodicIncompressibleGrid(config()), before = snapshot(g);
        assert.throws(() => g.step(bad));
        assert.throws(() => new p.PeriodicIncompressibleGrid(config(8, 6, {kinematicViscosity : bad})));
        assert.deepEqual(snapshot(g), before);
        g.delete();
    }
    for (const geometry of [{columns : 1.5}, {columns : -2}, {columns : NaN}, {rows : 2 ** 64}, {columns : 1},
                            {spacingX : 0}, {spacingY : Infinity}])
        assert.throws(() => new p.PeriodicIncompressibleGrid({...config(), geometry : {...config().geometry, ...geometry}}));
    assert.throws(() => new p.PeriodicIncompressibleGrid(config(8, 6, {density : 0})));
    assert.throws(() => new p.PeriodicIncompressibleGrid({geometry : config().geometry, density : 1}), TypeError);
    const g = new p.PeriodicIncompressibleGrid(config()), tg = taylorGreen(config());
    g.setVelocities(tg.xFaces, tg.yFaces);
    const before = snapshot(g);
    for (const options of [{cflSafety : 1}, {cflSafety : 0}, {cflSafety : NaN},
                           {maximumSubsteps : 10001}, {maximumIterations : 1000001}, {maximumCellVisits : 1000000001},
                           {maximumSubsteps : 1.5}, {maximumIterations : -1}, {maximumCellVisits : NaN},
                           {maximumSubsteps : Infinity}, {maximumIterations : 2 ** 53 + 2},
                           {absoluteDivergenceTolerance : -1}, {relativeVelocityTolerance : NaN},
                           {absoluteVelocityTolerance : Infinity}])
        assert.throws(() => g.step(.1, {...defaults, ...options}), JSON.stringify(options));
    const {maximumCellVisits, ...incomplete} = defaults;
    assert.throws(() => g.step(.1, incomplete), TypeError);
    assert.deepEqual(snapshot(g), before);
    // Velocity range and scale controls.
    const n = 48;
    for (const scale of [1e200, 1e-200]) {
        g.setVelocities(Array(n).fill(scale), Array(n).fill(0));
        const scaled = snapshot(g);
        assert.throws(() => g.step(.1));
        assert.deepEqual(snapshot(g), scaled);
    }
    // Every array shape is checked before any entry is read.
    let reads = 0;
    const counted = Array(n).fill(0);
    Object.defineProperty(counted, 0, {get() { ++reads; return 0; }});
    for (const [x, y, type] of [[counted, Array(n - 1).fill(0), RangeError], [{length : n}, Array(n).fill(0), TypeError],
                                [Array(n), Array(n).fill(0), TypeError], [Array(n).fill('1'), Array(n).fill(0), TypeError],
                                [Array(n).fill(0), Array(n).fill(NaN), TypeError], [Array(n).fill(0), Array(n).fill(Infinity), TypeError]]) {
        const current = snapshot(g);
        assert.throws(() => g.setVelocities(x, y), type);
        assert.deepEqual(snapshot(g), current);
    }
    assert.equal(reads, 0);
    g.delete();
    // Clock range is explicit.
    const clock = new p.PeriodicIncompressibleGrid(config(2, 2));
    clock.step(1e308);
    const previous = snapshot(clock);
    assert.throws(() => clock.step(1), /clock|unrepresentable/i);
    assert.deepEqual(snapshot(clock), previous);
    assert.equal(clock.time(), 1e308);
    clock.delete();
}

function smoke(p) {
    interfaceAndOwnership(p);
    denseCoupledOracle(p);
    transportedShear(p);
    const spatial = taylorGreenSpatial(p);
    const temporal = taylorGreenTemporal(p);
    ledgersAndResidual(p);
    budgetsAndRollback(p);
    invalidControls(p);
    console.log(`PASS: owned incompressible flow; independent dense projection/flux/BE oracles on 2x3/3x2/4x3 anisotropic axes, `
        + `transported-shear multipliers, Taylor-Green continuum spatial [velocity, pressure, energy] ${JSON.stringify(spatial)}, `
        + `fixed-grid temporal [velocity, pressure] ${JSON.stringify(temporal)}, ledgers/residual work, budgets/late rollback, range/clock/replay`);
}

function stress(p, probe) {
    const c = config(2, 2, {kinematicViscosity : .1}), g = new p.PeriodicIncompressibleGrid(c);
    const source = {xFaces : [.3, -.1, .2, .4], yFaces : [-.2, .1, .5, 0]};
    g.setVelocities(source.xFaces, source.yFaces);
    g.step(.01);
    // Batches restore these velocities and publish zero steps; the clock and
    // operator snapshots from the real step must survive every batch.
    g.setVelocities(source.xFaces, source.yFaces);
    g.step(0);
    const saved = snapshot(g), error = new Error('incompressible foreign failure'),
          stack = () => p._emscripten_stack_get_current();
    // Sizing uses the captured native observer, not shadowed JS methods.
    const nativeConfig = p.PeriodicIncompressibleGrid.prototype.config;
    p.PeriodicIncompressibleGrid.prototype.config = () => ({geometry : {columns : 1e20, rows : 1e20}});
    g.config = () => ({geometry : {columns : 1e20, rows : 1e20}});
    const failures = [
        () => g.step(-1), () => g.step(.2, {...defaults, maximumSubsteps : 0}),
        () => g.step(.2, {...defaults, maximumCellVisits : 40}),
        () => g.step(0, {...defaults, maximumCellVisits : 11}),
        () => g.step(.01, {...defaults, maximumSubsteps : 2 ** 32 + 1}),
        () => g.step(.01, {...defaults, cflSafety : 1}),
        () => g.setVelocities(source.xFaces, [0, 0, 0]),
        () => g.setVelocities(source.xFaces, Array(4)),
        () => g.setVelocities([0, 0, NaN, 0], source.yFaces)
    ].map(fn => [fn]);
    for (const which of [0, 1]) {
        failures.push([() => {
            const a = (which ? source.yFaces : source.xFaces).slice();
            Object.defineProperty(a, 3, {get() { throw error; }});
            which ? g.setVelocities(source.xFaces, a) : g.setVelocities(a, source.yFaces);
        }, error]);
        failures.push([() => {
            const a = new Proxy(source.xFaces, {get(target, key) { if (key === '2') throw error; return target[key]; }});
            which ? g.setVelocities(source.xFaces, a) : g.setVelocities(a, source.yFaces);
        }, error]);
    }
    for (const key of Object.keys(defaults))
        failures.push([() => g.step(.01, {...defaults, get [key]() { throw error; }}), error]);
    for (const value of [73, undefined, null, 'primitive'])
        failures.push([() => {
            const a = source.xFaces.slice();
            Object.defineProperty(a, 1, {get() { throw value; }});
            g.setVelocities(a, source.yFaces);
        }, value]);
    failures.push([() => new p.PeriodicIncompressibleGrid({...c, geometry : {...c.geometry, get rows() { throw error; }}}), error]);
    failures.push([() => new p.PeriodicIncompressibleGrid({...c, get kinematicViscosity() { throw error; }}), error]);
    let total = 0;
    function batch() {
        for (const item of failures) {
            const before = stack();
            assert.throws(item[0], e => item.length === 2 ? e === item[1] : e instanceof Error && !('excPtr' in e));
            assert.equal(stack(), before);
            ++total;
        }
        let reads = 0;
        const a = source.xFaces.slice();
        Object.defineProperty(a, 0, {get() { ++reads; throw error; }});
        assert.throws(() => g.setVelocities(a, []), e => e instanceof RangeError);
        assert.equal(reads, 0);
        // Failed nested calls surface as owned errors, not C++ exceptions.
        const nested = source.yFaces.slice();
        Object.defineProperty(nested, 3, {get() { g.step(-1); }});
        assert.throws(() => g.setVelocities(source.xFaces, nested), e => e instanceof Error && !('excPtr' in e));
        assert.throws(() => g.step(.01, {...defaults, get maximumCellVisits() { g.step(-1); }}),
                      e => e instanceof Error && !('excPtr' in e));
        // Successful reentrant calls leave the outer call valid.
        const reentrant = source.yFaces.slice();
        Object.defineProperty(reentrant, 3, {get() {
            assert.equal(probe.method(), 7);
            g.step(0);
            return source.yFaces[3];
        }});
        g.setVelocities(source.xFaces, reentrant);
        g.step(0, {...defaults, get cflSafety() { assert.equal(probe.method(), 7); return .8; }});
        for (const callback of ['entry', 'options', 'duration', 'nested']) {
            const receiver = new p.PeriodicIncompressibleGrid(c);
            receiver.setVelocities(source.xFaces, source.yFaces);
            const retained = receiver.clone(), before = snapshot(retained);
            let coercions = 0;
            if (callback === 'entry') {
                const x = source.xFaces.slice();
                Object.defineProperty(x, 2, {get() { receiver.delete(); return source.xFaces[2]; }});
                assert.throws(() => receiver.setVelocities(x, source.yFaces), /deleted/i);
            }
            if (callback === 'options')
                assert.throws(() => receiver.step(.01, {...defaults, get maximumIterations() {
                    receiver.delete(); return defaults.maximumIterations;
                }}), /deleted/i);
            if (callback === 'duration')
                assert.throws(() => receiver.step({valueOf() { ++coercions; receiver.delete(); return .01; }}),
                              e => coercions ? /deleted/i.test(e.message) : e instanceof TypeError);
            if (callback === 'nested')
                assert.throws(() => receiver.step(.01, {...defaults, get relativeVelocityTolerance() {
                    retained.step(0); receiver.delete(); return 1e-10;
                }}), /deleted/i);
            if (!receiver.isDeleted()) receiver.delete();
            assert.ok(coercions <= 1);
            const after = snapshot(retained);
            if (callback === 'nested') { before.lastStep = after.lastStep; } // The retained alias ran its own zero step.
            assert.deepEqual(after, before);
            retained.delete();
        }
    }
    try {
        for (let i = 0; i < 20; ++i) batch();
        total = 0;
        const before = p.boundaryTestStats(), savedStack = stack();
        for (let i = 0; i < 1000; ++i) batch();
        assert.deepEqual(p.boundaryTestStats(), before);
        assert.equal(stack(), savedStack);
        delete g.config;
        assert.deepEqual({...snapshot(g), config : c}, {...saved, config : c});
        console.log(`PASS: incompressible boundary stress; 1000 batches/${total} listed rejections plus array/options/duration `
            + `reentrancy/deletion/retained aliases; stack=${savedStack}, live heap=${before.heap}, uncaught=${before.uncaught}, emval=${before.emvalHandles}`);
    } finally {
        p.PeriodicIncompressibleGrid.prototype.config = nativeConfig;
        g.delete();
    }
}
module.exports = {smoke, stress};
