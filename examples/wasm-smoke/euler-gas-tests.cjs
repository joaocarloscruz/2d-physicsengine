const assert = require('node:assert/strict');
const config = {
    columns : 2,
    rows : 2,
    spacingX : 1,
    spacingY : 1,
    gamma : 1.4
};
const options = {
    cflSafety : .9,
    maxSubstep : .1,
    maximumSubsteps : 10000,
    maximumCellVisits : 100000000
};
const fields = [ 'density', 'momentumX', 'momentumY', 'totalEnergy' ];
const summaryKeys =
    'mass momentumX momentumY totalEnergy absoluteMomentumX absoluteMomentumY internalEnergy kineticEnergy minimumDensity maximumDensity minimumPressure maximumPressure'
        .split(' ');
const diagnosticKeys =
    'initial final massDefect momentumXDefect momentumYDefect totalEnergyDefect massRoundoffAllowance momentumXRoundoffAllowance momentumYRoundoffAllowance totalEnergyRoundoffAllowance duration timeBefore timeAfter lastSubstep maximumSignalSpeedX maximumSignalSpeedY maximumCfl substeps cellVisits zeroDurationNoOp'
        .split(' ');
const close = (a, b, t, label = '') =>
    assert.ok(Math.abs(a - b) <= t, `${label}: ${a} != ${b}; margin ${t}`);
const clone = x => structuredClone(x);
const cell = (s, k) => fields.map(f => s[f][k]);
function put(s, k, q) {
    fields.forEach((f, a) => { s[f][k] = q[a]; });
}
function conserved(r, u, v, p, gamma = 1.4) {
    return [ r, r * u, r * v, p / (gamma - 1) + .5 * r * (u * u + v * v) ];
}
function snapshot(g) {
    return {
        state : g.state(),
        primitives : g.primitives(),
        lastStep : g.lastStep(),
        time : g.time()
    };
}
function flux(q, x, gamma) {
    const u = q[1] / q[0], v = q[2] / q[0],
          p = (gamma - 1) * (q[3] - (q[1] * q[1] + q[2] * q[2]) / (2 * q[0]));
    return x ? [ q[1], q[1] * u + p, q[2] * u, (q[3] + p) * u ]
             : [ q[2], q[1] * v, q[2] * v + p, (q[3] + p) * v ];
}
// Independent convex combination of LF split states, rather than paired face
// accumulation or the production normalized transfers.
function oracle(s, c, h) {
    let ax = 0, ay = 0;
    for (let k = 0; k < s.density.length; ++k) {
        const q = cell(s, k), u = q[1] / q[0], v = q[2] / q[0];
        const p = (c.gamma - 1) * (q[3] - (q[1] * q[1] + q[2] * q[2]) / (2 * q[0])),
              sound = Math.sqrt(c.gamma * p / q[0]);
        ax = Math.max(ax, Math.abs(u) + sound);
        ay = Math.max(ay, Math.abs(v) + sound);
    }
    const cx = h * ax / c.spacingX, cy = h * ay / c.spacingY, out = clone(s);
    for (let j = 0; j < c.rows; ++j)
        for (let i = 0; i < c.columns; ++i) {
            const k = i + c.columns * j, value = cell(s, k).map(x => x * (1 - cx - cy));
            for (const x of [true, false])
                for (const sign of [-1, 1]) {
                    const neighbor = x ? (i + c.columns + sign) % c.columns + c.columns * j
                                       : i + c.columns * ((j + c.rows + sign) % c.rows);
                    const q = cell(s, neighbor), f = flux(q, x, c.gamma), alpha = x ? ax : ay,
                          weight = (x ? cx : cy) / 2;
                    for (let a = 0; a < 4; ++a)
                        value[a] += weight * (q[a] - sign * f[a] / alpha);
                }
            put(out, k, value);
        }
    return out;
}
function summary(s, c) {
    const d = Object.fromEntries(summaryKeys.map(k => [k, 0])), area = c.spacingX * c.spacingY;
    d.minimumDensity = d.minimumPressure = Infinity;
    d.maximumDensity = d.maximumPressure = -Infinity;
    for (let k = 0; k < s.density.length; ++k) {
        const q = cell(s, k), K = (q[1] * q[1] + q[2] * q[2]) / (2 * q[0]), I = q[3] - K,
              p = (c.gamma - 1) * I;
        assert.ok(q[0] > 0 && I > 0 && p > 0);
        for (let a = 0; a < 4; ++a)
            d[[ 'mass', 'momentumX', 'momentumY', 'totalEnergy' ][a]] += area * q[a];
        d.absoluteMomentumX += area * Math.abs(q[1]);
        d.absoluteMomentumY += area * Math.abs(q[2]);
        d.internalEnergy += area * I;
        d.kineticEnergy += area * K;
        d.minimumDensity = Math.min(d.minimumDensity, q[0]);
        d.maximumDensity = Math.max(d.maximumDensity, q[0]);
        d.minimumPressure = Math.min(d.minimumPressure, p);
        d.maximumPressure = Math.max(d.maximumPressure, p);
    }
    return d;
}
function audit(before, after, d, c) {
    assert.deepEqual(Object.keys(d).sort(), diagnosticKeys.slice().sort());
    for (const name of ['initial', 'final'])
        assert.deepEqual(Object.keys(d[name]).sort(), summaryKeys.slice().sort());
    const expected = [ summary(before, c), summary(after, c) ];
    for (let index = 0; index < 2; ++index) {
        const s = d[index ? 'final' : 'initial'];
        for (const key of summaryKeys) {
            assert.ok(Number.isFinite(s[key]));
            const scale = key === 'momentumX'   ? expected[index].absoluteMomentumX
                          : key === 'momentumY' ? expected[index].absoluteMomentumY
                                                : Math.abs(expected[index][key]);
            close(s[key], expected[index][key], 2e-12 * scale, key);
        }
    }
    for (const [key, defect, allowance] of [
             [ 'mass', 'massDefect', 'massRoundoffAllowance' ],
             [ 'momentumX', 'momentumXDefect', 'momentumXRoundoffAllowance' ],
             [ 'momentumY', 'momentumYDefect', 'momentumYRoundoffAllowance' ],
             [ 'totalEnergy', 'totalEnergyDefect', 'totalEnergyRoundoffAllowance' ]]) {
        close(d[defect], d.final[key] - d.initial[key], 0);
        assert.ok(Math.abs(d[defect]) <= d[allowance]);
    }
    for (const key of diagnosticKeys)
        if (typeof d[key] === 'number')
            assert.ok(Number.isFinite(d[key]), key);
    assert.ok(d.maximumCfl < 1);
    assert.equal(d.cellVisits,
                 (d.zeroDurationNoOp ? 3 : 4 + 6 * d.substeps) * before.density.length);
    assert.ok(d.final.minimumDensity > 0 && d.final.minimumPressure > 0);
}
const sinc = x => x === 0 ? 1 : Math.sin(x) / x;
function contact(p, nx) {
    const c = {...config, columns : nx, rows : nx / 2, spacingX : 1 / nx, spacingY : 2 / nx},
          g = new p.PeriodicEulerGasGrid(c), s = g.state();
    const u = .7, v = -.2, t = .15,
          factor = sinc(Math.PI * c.spacingX) * sinc(2 * Math.PI * c.spacingY);
    for (let j = 0; j < c.rows; ++j)
        for (let i = 0; i < nx; ++i) {
            const phase = 2 * Math.PI * ((i + .5) * c.spacingX + 2 * (j + .5) * c.spacingY);
            put(s, i + nx * j, conserved(1 + .2 * factor * Math.cos(phase), u, v, 1));
        }
    g.setState(s);
    const d = g.step(t), actual = g.state(), q = g.primitives();
    audit(s, actual, d, c);
    let error = 0;
    for (let j = 0; j < c.rows; ++j)
        for (let i = 0; i < nx; ++i) {
            const k = i + nx * j,
                  phase = 2 * Math.PI *
                          ((i + .5) * c.spacingX + 2 * (j + .5) * c.spacingY - (u + 2 * v) * t);
            error += (actual.density[k] - (1 + .2 * factor * Math.cos(phase))) ** 2;
            close(q.pressure[k], 1, 2e-13);
            close(q.velocityX[k], u, 2e-13);
            close(q.velocityY[k], v, 2e-13);
        }
    g.delete();
    return Math.sqrt(error / s.density.length);
}
// Exact one-dimensional pressure matching for the fixed Sod initial states.
// It shares no numerical update, wave-bound or work code with the engine.
function sod() {
    const gamma = 1.4, rL = 1, pL = 1, rR = .125, pR = .1, beta = (gamma - 1) / (gamma + 1);
    const curve = (p, r, ps) => p > ps
                                    ? (p - ps) * Math.sqrt(2 / ((gamma + 1) * r) / (p + beta * ps))
                                    : 2 * Math.sqrt(gamma * ps / r) / (gamma - 1) *
                                          ((p / ps) ** ((gamma - 1) / (2 * gamma)) - 1);
    let lo = pR, hi = pL;
    for (let i = 0; i < 80; ++i) {
        const p = (lo + hi) / 2;
        if (curve(p, rL, pL) + curve(p, rR, pR) > 0)
            hi = p;
        else
            lo = p;
    }
    const ps = (lo + hi) / 2, u = (curve(ps, rR, pR) - curve(ps, rL, pL)) / 2,
          rsl = (ps / pL) ** (1 / gamma), rsr = rR * ((ps / pR + beta) / (beta * ps / pR + 1));
    const head = -Math.sqrt(gamma * pL / rL), tail = u - Math.sqrt(gamma * ps / rsl),
          shock = Math.sqrt(gamma * pR / rR) *
                  Math.sqrt((gamma + 1) / (2 * gamma) * ps / pR + (gamma - 1) / (2 * gamma));
    close(ps, .303130178050647, 2e-15);
    close(u, .92745262004895, 3e-15);
    const sample = xi => {
        if (xi < head)
            return conserved(rL, 0, 0, pL);
        if (xi < tail) {
            const cl = -head, v = 2 / (gamma + 1) * (cl + xi),
                  sound = 2 / (gamma + 1) * (cl - .5 * (gamma - 1) * xi);
            return conserved(rL * (sound / cl) ** (2 / (gamma - 1)), v, 0,
                             pL * (sound / cl) ** (2 * gamma / (gamma - 1)));
        }
        if (xi < u)
            return conserved(rsl, u, 0, ps);
        if (xi < shock)
            return conserved(rsr, u, 0, ps);
        return conserved(rR, 0, 0, pR);
    };
    return {sample, speeds : [ head, tail, u, shock ]};
}
function average(left, right, fn) {
    const points = [ -.8611363115940526, -.3399810435848563, .3399810435848563, .8611363115940526 ],
          weights = [ .3478548451374539, .6521451548625461, .6521451548625461, .3478548451374539 ],
          value = [ 0, 0, 0, 0 ];
    points.forEach((x, k) => {
        const q = fn((left + right) / 2 + (right - left) / 2 * x);
        for (let a = 0; a < 4; ++a)
            value[a] += .5 * weights[k] * q[a];
    });
    return value;
}
function shock(p, nx, length) {
    const c = {...config, columns : nx, rows : 2, spacingX : length / nx, spacingY : .5},
          g = new p.PeriodicEulerGasGrid(c), s = g.state(), t = .12, center = length / 2,
          exact = sod();
    for (let j = 0; j < 2; ++j)
        for (let i = 0; i < nx; ++i)
            put(s, i + nx * j,
                (i + .5) * c.spacingX < center ? conserved(1, 0, 0, 1) : conserved(.125, 0, 0, .1));
    g.setState(s);
    const d = g.step(t), actual = g.state();
    audit(s, actual, d, c);
    let error = 0;
    for (let i = 0; i < nx; ++i)
        if (Math.abs((i + .5) * c.spacingX - center) < .4) {
            const left = i * c.spacingX - center, right = (i + 1) * c.spacingX - center,
                  cuts =
                      [
                          left, right,
                          ...exact.speeds.map(x => x * t).filter(x => x > left && x < right)
                      ].sort((a, b) => a - b),
                  q = [ 0, 0, 0, 0 ];
            for (let k = 1; k < cuts.length; ++k) {
                const a = average(cuts[k - 1], cuts[k], x => exact.sample(x / t));
                for (let f = 0; f < 4; ++f)
                    q[f] += a[f] * (cuts[k] - cuts[k - 1]) / c.spacingX;
            }
            for (let f = 0; f < 4; ++f)
                error += c.spacingX * Math.abs(cell(actual, i)[f] - q[f]);
        }
    g.delete();
    return {error, state : actual};
}
function smoke(p) {
    const defaults = new p.PeriodicEulerGasGrid();
    assert.deepEqual(defaults.config(),
                     {columns : 16, rows : 16, spacingX : 1, spacingY : 1, gamma : 1.4});
    assert.equal(defaults.time(), 0);
    assert.equal(defaults.lastStep().cellVisits, 0);
    const base = defaults.state();
    const zero = defaults.step(0);
    audit(base, defaults.state(), zero, defaults.config());
    assert.ok(zero.zeroDurationNoOp);
    defaults.delete();
    for (const [nx, ny] of [[ 7, 5 ], [ 2, 5 ], [ 5, 2 ], [ 2, 2 ]]) {
        const c = {...config, columns : nx, rows : ny, spacingX : .7, spacingY : 1.3},
              g = new p.PeriodicEulerGasGrid(c), s = g.state();
        for (let k = 0; k < s.density.length; ++k)
            put(s, k,
                conserved(.9 + .15 * Math.sin(k), .3 * Math.cos(k), -.2 * Math.sin(2 * k),
                          1 + .1 * Math.cos(3 * k)));
        const expected = oracle(s, c, .001);
        g.setState(s);
        const d = g.step(.001), actual = g.state();
        assert.equal(d.substeps, 1);
        audit(s, actual, d, c);
        fields.forEach(f => actual[f].forEach((x, k) => close(x, expected[f][k], 2e-15)));
        const replay = new p.PeriodicEulerGasGrid(c);
        replay.setState(s);
        assert.deepEqual(replay.step(.001), d);
        assert.deepEqual(replay.state(), actual);
        replay.delete();
        g.delete();
    }
    const g = new p.PeriodicEulerGasGrid(config), state = g.state();
    fields.forEach((f, a) => state[f].fill(conserved(2, -.7, .2, 3)[a]));
    g.setState(state);
    const d =
        g.step(.01, {...options, maxSubstep : .01, maximumSubsteps : 1, maximumCellVisits : 40});
    assert.deepEqual(g.state(), state);
    audit(state, g.state(), d, config);
    const retained = snapshot(g), historical = g.lastStep(), changed = clone(state);
    changed.density.fill(2.1);
    g.setState(changed);
    assert.deepEqual(g.lastStep(), historical);
    assert.equal(g.time(), .01);
    const fresh = g.step(0, {...options, maximumSubsteps : 0, maximumCellVisits : 12});
    assert.equal(fresh.cellVisits, 12);
    assert.ok(fresh.final.mass > historical.final.mass);
    assert.deepEqual(g.state(), changed);
    const kept = snapshot(g);
    const cfg = g.config(), s = g.state(), q = g.primitives(), last = g.lastStep();
    cfg.gamma = 999;
    fields.forEach(f => s[f][0] = 999);
    Object.values(q).forEach(a => a[0] = 999);
    last.final.mass = 999;
    assert.deepEqual(snapshot(g), kept);
    for (const fail of [() => g.step(-1), () => g.step(NaN), () => g.step(Infinity),
                       () => g.step(.2, {...options, maximumSubsteps : 0}),
                       () => g.step(.2, {...options, maximumSubsteps : 1}),
                       () => g.step(.2, {...options, maxSubstep : .001, maximumCellVisits : 40}),
                       () => g.step(0, {...options, maximumCellVisits : 11}),
                       () => g.setState({...changed, totalEnergy : [ 1, 1, 1, 0 ]})]) {
        assert.throws(fail);
        assert.deepEqual(snapshot(g), kept);
    }
    g.delete();
    assert.equal(retained.state.density[0], 2);
    assert.equal(retained.lastStep.final.mass, d.final.mass);
    const errors = [ 32, 64, 128 ].map(n => contact(p, n));
    assert.ok(errors[1] < .7 * errors[0] && errors[2] < .65 * errors[1] &&
              errors[1] / errors[2] > 1.65);
    const shocks = [ 128, 256, 512 ].map(n => shock(p, n, 2));
    assert.ok(shocks[1].error < .8 * shocks[0].error && shocks[2].error < .8 * shocks[1].error &&
              shocks[2].error < .08);
    const control = shock(p, 512, 4);
    let difference = 0;
    for (let i = 0; i < 256; ++i)
        if (Math.abs((i + .5) * 2 / 256 - 1) < .4)
            for (let f = 0; f < 4; ++f)
                difference +=
                    2 / 256 *
                    Math.abs(cell(shocks[1].state, i)[f] - cell(control.state, i + 128)[f]);
    assert.ok(difference < 1e-5 * shocks[1].error);
    // A one-step fixture close to the strict multidimensional positivity bound.
    // Its independent physical speeds set the duration; native outward rounding
    // may make the reported CFL slightly larger, but must keep it strictly <1.
    for (const gamma of [1.01, 1.4, 3, 20]) {
        const c = {...config, columns : 2, rows : 5, spacingX : .3, spacingY : 1.7, gamma};
        const gas = new p.PeriodicEulerGasGrid(c), s = gas.state();
        let ax = 0, ay = 0;
        for (let k = 0; k < s.density.length; ++k) {
            const r = .2 + .1 * k, u = .3 * Math.sin(k), v = -.2 * Math.cos(k),
                  pressure = .3 + .05 * k;
            put(s, k, conserved(r, u, v, pressure, gamma));
            const sound = Math.sqrt(gamma * pressure / r);
            ax = Math.max(ax, Math.abs(u) + sound);
            ay = Math.max(ay, Math.abs(v) + sound);
        }
        const duration = .999 / (ax / c.spacingX + ay / c.spacingY);
        gas.setState(s);
        const d = gas.step(duration, {
            ...options,
            cflSafety : .9999999999999999,
            maxSubstep : duration,
            maximumSubsteps : 1
        });
        assert.equal(d.substeps, 1);
        audit(s, gas.state(), d, c);
        gas.delete();
    }
    for (const gamma of [1.01, 1.4, 3, 20])
        for (const scale of [1e-150, 1, 1e150]) {
            const c = {...config, columns : 4, rows : 2, spacingX : .3, spacingY : .7, gamma},
                  gas = new p.PeriodicEulerGasGrid(c), s = gas.state();
            for (let k = 0; k < s.density.length; ++k)
                put(s, k,
                    conserved(scale * (1 + .1 * k), .02 * Math.sin(k), .02 * Math.cos(k),
                              scale * (.3 + .01 * k), gamma));
            gas.setState(s);
            const dd = gas.step(.001);
            assert.ok(dd.final.minimumDensity > 0 && dd.final.minimumPressure > 0 &&
                      dd.maximumCfl < 1);
            assert.ok(Math.abs(dd.totalEnergyDefect) <= dd.totalEnergyRoundoffAllowance);
            gas.delete();
        }
    const large = new p.PeriodicEulerGasGrid(
              {...config, columns : 4, spacingX : 1e-154, spacingY : 1e-154}),
          ls = large.state();
    ls.density.fill(1.5e308);
    ls.totalEnergy.fill(1.7e308);
    ls.momentumX = ls.momentumX.map((_, k) => (k % 4 < 2 ? 1 : -1) * .75e308);
    large.setState(ls);
    large.step(0);
    const lp = snapshot(large);
    assert.throws(() => large.step(4e-155));
    assert.deepEqual(snapshot(large), lp);
    large.delete();
    const cold = new p.PeriodicEulerGasGrid({...config, spacingX : 1e154, spacingY : 1e154}),
          cs = cold.state();
    cs.density.fill(.1);
    cs.totalEnergy.fill(1e-320);
    cold.setState(cs);
    cold.step(1e308, {...options, maxSubstep : 1e308, maximumSubsteps : 1});
    const cp = snapshot(cold);
    for (const duration of [1, 1e308]) {
        assert.throws(() => cold.step(duration, {...options, maxSubstep : 1e308}));
        assert.deepEqual(snapshot(cold), cp);
    }
    cold.delete();
    for (const f of ['columns', 'rows'])
        for (const bad of [-1, 0, 1, .5, NaN, Infinity, 262145, 2 ** 32, 2 ** 32 + 1])
            assert.throws(() => new p.PeriodicEulerGasGrid({...config, [f] : bad}));
    assert.throws(() => new p.PeriodicEulerGasGrid({...config, columns : 512, rows : 513}));
    assert.throws(() => new p.PeriodicEulerGasGrid({columns : 2}));
    for (const f of ['spacingX', 'spacingY'])
        for (const bad of [0, -1, NaN, Infinity])
            assert.throws(() => new p.PeriodicEulerGasGrid({...config, [f] : bad}));
    for (const bad of [0, 1, NaN, Infinity])
        assert.throws(() => new p.PeriodicEulerGasGrid({...config, gamma : bad}));
    const validation = new p.PeriodicEulerGasGrid(config);
    for (const f of Object.keys(options)) {
        const incomplete = {...options};
        delete incomplete[f];
        assert.throws(() => validation.step(.01, incomplete));
        for (const bad of f.startsWith('maximum') ? [ -1, .5, NaN, Infinity, 2 ** 32, 2 ** 32 + 1 ]
                                                  : [ 0, -1, NaN, Infinity ])
            assert.throws(() => validation.step(.01, {...options, [f] : bad}));
    }
    assert.throws(() => validation.step(.01, {...options, cflSafety : 1}));
    assert.throws(() => validation.step(.01, {...options, maximumSubsteps : 1000001}));
    assert.throws(() => validation.step(.01, {...options, maximumCellVisits : 1000000001}));
    for (const f of fields)
        for (const bad
                 of [[], Array(4), new Float64Array(4), [ 1, 1, 1, Infinity ], [ 1, 1, 1, '1' ]])
            assert.throws(() => validation.setState({...validation.state(), [f] : bad}));
    validation.delete();
    console.log(
        `PASS: owned Euler gas; independent split-state/two-axis/two-cell updates, contact RMS ${
            JSON.stringify(errors)}, exact Sod errors ${
            JSON.stringify(shocks.map(x => x.error))}, image difference ${
            difference}, physical summaries/positivity/range/budgets/clock/ownership/replay`);
}
function stress(p, probe) {
    const g = new p.PeriodicEulerGasGrid(config), source = g.state();
    g.step(0);
    const saved = snapshot(g), error = new Error('Euler foreign failure'),
          stack = () => p._emscripten_stack_get_current();
    const nativeConfig = p.PeriodicEulerGasGrid.prototype.config;
    p.PeriodicEulerGasGrid.prototype.config = () => ({columns : 1e20, rows : 1e20});
    g.config = () => ({columns : 1e20, rows : 1e20});
    const failures = [
        () => g.step(-1), () => g.step(.2, {...options, maximumSubsteps : 0}),
        () => g.step(.2, {...options, maxSubstep : .001, maximumCellVisits : 40}),
        () => g.step(0, {...options, maximumCellVisits : 11}),
        () => g.step(.01, {...options, maximumSubsteps : 2 ** 32 + 1}),
        () => g.setState({...source, totalEnergy : [ 1, 1, 1, 0 ]})
    ].map(fn => [fn]);
    for (const f of fields) {
        failures.push([ () => g.setState({...source, get[f]() { throw error; }}), error ]);
        failures.push([
            () => {
                const a = source[f].slice();
                Object.defineProperty(a, 3, {get() { throw error; }});
                g.setState({...source, [f] : a});
            },
            error
        ]);
        failures.push([ () => g.setState({...source, [f] : Array(4)}) ]);
    }
    for (const key of Object.keys(options))
        failures.push([ () => g.step(.01, {...options, get[key]() { throw error; }}), error ]);
    failures.push([
        () => g.setState(new Proxy(source, {
            get(target, key) {
                if (key === 'momentumY')
                    throw error;
                return target[key];
            }
        })),
        error
    ]);
    failures.push([
        () => {
            const a = new Proxy(source.density, {
                get(target, key) {
                    if (key === '3')
                        throw error;
                    return target[key];
                }
            });
            g.setState({...source, density : a});
        },
        error
    ]);
    for (const value of [73, undefined, null, 'primitive'])
        failures.push([
            () => {
                const a = source.totalEnergy.slice();
                Object.defineProperty(a, 3, {get() { throw value; }});
                g.setState({...source, totalEnergy : a});
            },
            value
        ]);
    failures.push(
        [ () => new p.PeriodicEulerGasGrid({...config, get gamma() { throw error; }}), error ]);
    let total = 0;
    function batch() {
        for (const item of failures) {
            const before = stack();
            assert.throws(item[0], e => item.length === 2 ? e === item[1]
                                                          : e instanceof Error && !('excPtr' in e));
            assert.equal(stack(), before);
            ++total;
        }
        let reads = 0;
        const a = source.density.slice();
        Object.defineProperty(a, 0, {
            get() {
                ++reads;
                throw error;
            }
        });
        assert.throws(() => g.setState({...source, density : a, totalEnergy : []}),
                      e => e instanceof RangeError);
        assert.equal(reads, 0);
        const nested = source.totalEnergy.slice();
        Object.defineProperty(nested, 3, {get() { g.step(-1); }});
        assert.throws(() => g.setState({...source, totalEnergy : nested}),
                      e => e instanceof Error && !('excPtr' in e));
        assert.throws(() => g.step(.01, {...options, get maximumCellVisits() { g.step(-1); }}),
                      e => e instanceof Error && !('excPtr' in e));
        const reentrant = source.totalEnergy.slice();
        Object.defineProperty(reentrant, 3, {
            get() {
                assert.equal(probe.method(), 7);
                g.step(0);
                return source.totalEnergy[3];
            }
        });
        g.setState({...source, totalEnergy : reentrant});
        g.step(0, {
            ...options,
            get maxSubstep() {
                assert.equal(probe.method(), 7);
                return .1;
            }
        });
        for (const callback of ['entry', 'field', 'options', 'duration']) {
            const receiver = new p.PeriodicEulerGasGrid(config);
            const retained = receiver.clone(), before = snapshot(retained);
            let coercions = 0;
            if (callback === 'entry') {
                const a = source.totalEnergy.slice();
                Object.defineProperty(a, 3, {
                    get() {
                        receiver.delete();
                        return source.totalEnergy[3];
                    }
                });
                assert.throws(() => receiver.setState({...source, totalEnergy : a}), /deleted/i);
            }
            if (callback === 'field')
                assert.throws(() => receiver.setState({
                    ...source,
                    get totalEnergy() {
                        receiver.delete();
                        return source.totalEnergy;
                    }
                }),
                              /deleted/i);
            if (callback === 'options')
                assert.throws(() => receiver.step(.01, {
                    ...options,
                    get maximumCellVisits() {
                        receiver.delete();
                        return options.maximumCellVisits;
                    }
                }),
                              /deleted/i);
            if (callback === 'duration')
                assert.throws(() => receiver.step({
                    valueOf() {
                        ++coercions;
                        receiver.delete();
                        return .01;
                    }
                }),
                              e => coercions ? /deleted/i.test(e.message) : e instanceof TypeError);
            if (!receiver.isDeleted())
                receiver.delete();
            assert.ok(coercions <= 1);
            assert.deepEqual(snapshot(retained), before);
            retained.delete();
        }
    }
    try {
        for (let i = 0; i < 20; ++i)
            batch();
        total = 0;
        const before = p.boundaryTestStats(), savedStack = stack();
        for (let i = 0; i < 1000; ++i)
            batch();
        assert.deepEqual(p.boundaryTestStats(), before);
        assert.equal(stack(), savedStack);
        assert.deepEqual(snapshot(g), saved);
        console.log(`PASS: Euler boundary stress; 1000 batches/${
            total} listed rejections plus array/options/duration reentrancy/deletion; stack=${
            savedStack}, live heap=${before.heap}, uncaught=${before.uncaught}`);
    } finally {
        p.PeriodicEulerGasGrid.prototype.config = nativeConfig;
        g.delete();
    }
}
module.exports = {
    smoke,
    stress
};
