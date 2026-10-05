const assert = require('node:assert/strict');

const fields = ['density', 'momentumX', 'momentumY', 'totalEnergy'];
const config = {columns: 2, rows: 2, spacingX: .5, spacingY: .7, gamma: 1.4};
const options = {cflSafety: .9, maxSubstep: .1, maximumSubsteps: 10000,
    maximumCellVisits: 100000000, maximumAttempts: 20000, maximumRetriesPerSubstep: 16};
const counts = ['attempts', 'rejectedAttempts', 'reconstructionPreparations',
    'reconstructionTrials', 'limitedSlopeCells', 'positivityLimitedCells',
    'rangeLimitedCells', 'zeroSlopeFallbackCells', 'forwardEulerStages', 'blendPasses'];
const extended = [...counts, 'minimumSlopeScale', 'maximumRejectedCfl'];
const baseFields = ['initial', 'final', 'massDefect', 'momentumXDefect', 'momentumYDefect',
    'totalEnergyDefect', 'massRoundoffAllowance', 'momentumXRoundoffAllowance',
    'momentumYRoundoffAllowance', 'totalEnergyRoundoffAllowance', 'duration', 'timeBefore',
    'timeAfter', 'lastSubstep', 'maximumSignalSpeedX', 'maximumSignalSpeedY', 'maximumCfl',
    'substeps', 'cellVisits', 'zeroDurationNoOp'];
const summaryFields = ['mass', 'momentumX', 'momentumY', 'totalEnergy', 'absoluteMomentumX',
    'absoluteMomentumY', 'internalEnergy', 'kineticEnergy', 'minimumDensity', 'maximumDensity',
    'minimumPressure', 'maximumPressure'];
const copy = x => structuredClone(x);
const at = (s, k) => fields.map(f => s[f][k]);
const put = (s, k, q) => fields.forEach((f, a) => s[f][k] = q[a]);
const state = n => Object.fromEntries(fields.map(f => [f, Array(n).fill(0)]));
const conserved = (rho, u, v, pressure, gamma = 1.4) =>
    [rho, rho * u, rho * v, pressure / (gamma - 1) + .5 * rho * (u * u + v * v)];
function close(a, b, tolerance = 5e-14) {
    assert.ok(Math.abs(a - b) <= tolerance * Math.max(Math.abs(a), Math.abs(b), 1e-300),
        `${a} vs ${b}, relative tolerance ${tolerance}`);
}
function closeState(a, b, tolerance = 5e-15) {
    fields.forEach(f => a[f].forEach((x, k) => assert.ok(Math.abs(x - b[f][k]) <= tolerance,
        `${f}[${k}]: ${x} vs ${b[f][k]}, absolute tolerance ${tolerance}`)));
}
function snapshot(g) {
    return {state: g.state(), primitives: g.primitives(), time: g.time(),
        lastStep: g.lastStep(), lastSecondOrderStep: g.lastSecondOrderStep()};
}
function base(d) {
    return Object.fromEntries(baseFields.map(f => [f, d[f]]));
}
function summary(s, c) {
    const d = Object.fromEntries(summaryFields.map(f => [f, 0]));
    d.minimumDensity = d.minimumPressure = Infinity;
    d.maximumDensity = d.maximumPressure = -Infinity;
    const area = c.spacingX * c.spacingY;
    for (let k = 0; k < s.density.length; ++k) {
        const [r, mx, my, e] = at(s, k), u = mx / r, v = my / r;
        const kinetic = .5 * r * (u * u + v * v), internal = e - kinetic;
        const pressure = (c.gamma - 1) * internal;
        assert.ok(r > 0 && internal > 0 && pressure > 0);
        for (const x of [r, mx, my, e, u, v, pressure, Math.sqrt(c.gamma * pressure / r)])
            assert.ok(Number.isFinite(x));
        d.mass += r * area; d.momentumX += mx * area; d.momentumY += my * area;
        d.totalEnergy += e * area; d.absoluteMomentumX += Math.abs(mx) * area;
        d.absoluteMomentumY += Math.abs(my) * area;
        d.internalEnergy += internal * area; d.kineticEnergy += kinetic * area;
        d.minimumDensity = Math.min(d.minimumDensity, r);
        d.maximumDensity = Math.max(d.maximumDensity, r);
        d.minimumPressure = Math.min(d.minimumPressure, pressure);
        d.maximumPressure = Math.max(d.maximumPressure, pressure);
    }
    return d;
}
function audit(s, actual, d, c) {
    assert.deepEqual(Object.keys(d).sort(), [...baseFields, ...extended].sort());
    for (const [name, input] of [['initial', s], ['final', actual]]) {
        assert.deepEqual(Object.keys(d[name]).sort(), summaryFields.slice().sort());
        const expected = summary(input, c);
        // Signed momenta can nearly cancel; their absolute integrals set the scale.
        for (const f of summaryFields) {
            const scale = f === 'momentumX' ? expected.absoluteMomentumX :
                f === 'momentumY' ? expected.absoluteMomentumY :
                f === 'internalEnergy' ? expected.totalEnergy :
                f.endsWith('Pressure') ? Math.max(...input.totalEnergy) * (c.gamma - 1) : Math.abs(expected[f]);
            assert.ok(Math.abs(d[name][f] - expected[f]) <= 2e-12 * Math.max(scale, 1e-300), f);
        }
    }
    for (const f of ['mass', 'momentumX', 'momentumY', 'totalEnergy']) {
        const defect = d[`${f}Defect`], allowance = d[`${f}RoundoffAllowance`];
        assert.ok(Math.abs(defect) <= allowance);
        close(defect, d.final[f] - d.initial[f], 1e-12);
    }
    const n = s.density.length;
    if (d.zeroDurationNoOp) {
        assert.equal(d.cellVisits, 3 * n);
        for (const f of counts) assert.equal(d[f], 0);
        assert.equal(d.minimumSlopeScale, 1);
        assert.equal(d.maximumRejectedCfl, 0);
        assert.equal(d.substeps, 0);
    } else {
        assert.equal(d.attempts, d.substeps + d.rejectedAttempts);
        assert.equal(d.reconstructionPreparations, 2 * d.attempts);
        assert.equal(d.forwardEulerStages, d.attempts + d.substeps);
        assert.equal(d.blendPasses, d.substeps);
        assert.ok(d.reconstructionTrials >= 2 * d.attempts * n);
        assert.ok(d.reconstructionTrials <= 34 * 2 * d.attempts * n);
        assert.equal(d.cellVisits, (4 + 8 * d.attempts + 5 * d.substeps) * n + d.reconstructionTrials);
        assert.ok(d.maximumCfl < .5);
    }
}

// Independent physical conserved MC reconstruction. No production normalization,
// transfer arithmetic, common-theta implementation, or native stepping is used.
function mc(l, r) {
    return l * r <= 0 ? 0 : Math.sign(l) * Math.min(2 * Math.abs(l), Math.abs(l + r) / 2, 2 * Math.abs(r));
}
function physicalFaces(s, c) {
    const faces = [], nx = c.columns, ny = c.rows;
    let ax = 0, ay = 0;
    for (let k = 0; k < s.density.length; ++k) {
        const i = k % nx, j = Math.floor(k / nx), q = at(s, k);
        const l = at(s, (i + nx - 1) % nx + nx * j), r = at(s, (i + 1) % nx + nx * j);
        const b = at(s, i + nx * ((j + ny - 1) % ny)), t = at(s, i + nx * ((j + 1) % ny));
        const sx = q.map((x, a) => mc(x - l[a], r[a] - x));
        const sy = q.map((x, a) => mc(x - b[a], t[a] - x));
        faces[k] = [q.map((x, a) => x - sx[a] / 2), q.map((x, a) => x + sx[a] / 2),
            q.map((x, a) => x - sy[a] / 2), q.map((x, a) => x + sy[a] / 2)];
        faces[k].forEach((qf, f) => {
            const u = qf[1] / qf[0], v = qf[2] / qf[0];
            const p = (c.gamma - 1) * (qf[3] - .5 * qf[0] * (u * u + v * v));
            assert.ok(qf[0] > 0 && p > 0, 'unlimited physical oracle requires admissible faces');
            const sound = Math.sqrt(c.gamma * p / qf[0]);
            if (f < 2) ax = Math.max(ax, Math.abs(u) + sound);
            else ay = Math.max(ay, Math.abs(v) + sound);
        });
    }
    return {faces, ax, ay};
}
function flux(q, x, gamma) {
    const u = q[1] / q[0], v = q[2] / q[0];
    const p = (gamma - 1) * (q[3] - .5 * q[0] * (u * u + v * v));
    return x ? [q[1], q[1] * u + p, q[2] * u, (q[3] + p) * u] :
        [q[2], q[1] * v, q[2] * v + p, (q[3] + p) * v];
}
function face(l, r, x, alpha, gamma) {
    const fl = flux(l, x, gamma), fr = flux(r, x, gamma);
    return l.map((q, a) => .5 * (fl[a] + fr[a] - alpha * (r[a] - q)));
}
function derivative(s, c) {
    const {faces: f, ax, ay} = physicalFaces(s, c), nx = c.columns, ny = c.rows;
    const result = state(s.density.length);
    // Cellwise flux divergence is independent of native shared-face accumulation.
    for (let k = 0; k < s.density.length; ++k) {
        const i = k % nx, j = Math.floor(k / nx);
        const l = (i + nx - 1) % nx + nx * j, r = (i + 1) % nx + nx * j;
        const b = i + nx * ((j + ny - 1) % ny), t = i + nx * ((j + 1) % ny);
        const fl = face(f[l][1], f[k][0], true, ax, c.gamma);
        const fr = face(f[k][1], f[r][0], true, ax, c.gamma);
        const fb = face(f[b][3], f[k][2], false, ay, c.gamma);
        const ft = face(f[k][3], f[t][2], false, ay, c.gamma);
        put(result, k, fl.map((q, a) => (q - fr[a]) / c.spacingX + (fb[a] - ft[a]) / c.spacingY));
    }
    return result;
}
function shift(s, d, h) {
    return Object.fromEntries(fields.map(f => [f, s[f].map((x, k) => x + h * d[f][k])]));
}
function rk2(s, c, h) {
    const s1 = shift(s, derivative(s, c), h), s2 = shift(s1, derivative(s1, c), h);
    return Object.fromEntries(fields.map(f => [f, s[f].map((x, k) => .5 * (x + s2[f][k]))]));
}
function rk4(s, c, duration, steps) {
    const h = duration / steps;
    for (let step = 0; step < steps; ++step) {
        const a = derivative(s, c), b = derivative(shift(s, a, h / 2), c);
        const cc = derivative(shift(s, b, h / 2), c), d = derivative(shift(s, cc, h), c);
        s = Object.fromEntries(fields.map(f => [f, s[f].map((x, k) =>
            x + h / 6 * (a[f][k] + 2 * b[f][k] + 2 * cc[f][k] + d[f][k]))]));
    }
    return s;
}
function rotate(s, nx, ny) {
    const result = state(nx * ny);
    for (let j = 0; j < ny; ++j) for (let i = 0; i < nx; ++i) {
        const q = at(s, i + nx * j);
        [q[1], q[2]] = [q[2], q[1]];
        put(result, j + ny * i, q);
    }
    return result;
}
const sinc = x => Math.sin(x) / x;
function contact(p, nx) {
    const c = {...config, columns: nx, rows: nx / 2, spacingX: 1 / nx, spacingY: 2 / nx};
    const s = state(c.columns * c.rows), t = .15, u = .7, v = -.2;
    const factor = sinc(Math.PI * c.spacingX) * sinc(2 * Math.PI * c.spacingY);
    for (let j = 0; j < c.rows; ++j) for (let i = 0; i < nx; ++i) {
        const phase = 2 * Math.PI * ((i + .5) * c.spacingX + 2 * (j + .5) * c.spacingY);
        put(s, i + nx * j, conserved(1 + .2 * factor * Math.cos(phase), u, v, 1));
    }
    const errors = {};
    for (const method of ['step', 'stepSecondOrder']) {
        const g = new p.PeriodicEulerGasGrid(c);
        try {
            g.setState(s);
            const d = g[method](t), actual = g.state(), q = g.primitives();
            if (method === 'stepSecondOrder') audit(s, actual, d, c);
            let error = 0;
            for (let j = 0; j < c.rows; ++j) for (let i = 0; i < nx; ++i) {
                const k = i + nx * j;
                const phase = 2 * Math.PI * ((i + .5) * c.spacingX + 2 * (j + .5) * c.spacingY - (u + 2 * v) * t);
                error += (actual.density[k] - 1 - .2 * factor * Math.cos(phase)) ** 2;
                close(q.pressure[k], 1, 2e-12); close(q.velocityX[k], u, 2e-12);
                close(q.velocityY[k], v, 2e-12);
            }
            errors[method] = Math.sqrt(error / s.density.length);
        } finally { g.delete(); }
    }
    return errors;
}
function simpleWave(x, t) {
    const gamma = 1.4, amplitude = .04, c0 = Math.sqrt(gamma);
    let lo = -amplitude, hi = amplitude;
    for (let i = 0; i < 70; ++i) {
        const u = (lo + hi) / 2;
        if (u - amplitude * Math.sin(2 * Math.PI * (x - (c0 + (gamma + 1) * u / 2) * t)) > 0) hi = u;
        else lo = u;
    }
    const u = (lo + hi) / 2, sound = c0 + (gamma - 1) * u / 2;
    const rho = (sound / c0) ** (2 / (gamma - 1));
    return conserved(rho, u, 0, rho ** gamma, gamma);
}
function average(l, r, sample) {
    const nodes = [-.8611363115940526, -.3399810435848563, .3399810435848563, .8611363115940526];
    const weights = [.3478548451374539, .6521451548625461, .6521451548625461, .3478548451374539];
    const q = [0, 0, 0, 0];
    nodes.forEach((x, k) => sample((l + r) / 2 + (r - l) * x / 2).forEach((v, a) => q[a] += weights[k] * v / 2));
    return q;
}
function waveState(c, t, phase = 0) {
    const s = state(c.columns * c.rows);
    for (let i = 0; i < c.columns; ++i) {
        const q = average(i * c.spacingX, (i + 1) * c.spacingX, x => simpleWave(x + phase, t));
        for (let j = 0; j < c.rows; ++j) put(s, i + c.columns * j, q);
    }
    return s;
}
function l1(a, b) {
    return fields.reduce((sum, f) => sum + a[f].reduce((q, x, k) => q + Math.abs(x - b[f][k]), 0), 0) / a.density.length;
}
function nonlinearSpatial(p, nx) {
    const c = {...config, columns: nx, spacingX: 1 / nx, spacingY: .5}, s = waveState(c, 0);
    const expected = waveState(c, .2), errors = {};
    for (const method of ['step', 'stepSecondOrder']) {
        const g = new p.PeriodicEulerGasGrid(c);
        try {
            g.setState(s); const d = g[method](.2);
            if (method === 'stepSecondOrder') audit(s, g.state(), d, c);
            errors[method] = l1(g.state(), expected);
            const cy = {...c, columns: c.rows, rows: c.columns, spacingX: c.spacingY, spacingY: c.spacingX};
            const y = new p.PeriodicEulerGasGrid(cy);
            try {
                const sy = rotate(s, c.columns, c.rows);
                y.setState(sy); const dy = y[method](.2);
                if (method === 'stepSecondOrder') audit(sy, y.state(), dy, cy);
                close(l1(y.state(), rotate(expected, c.columns, c.rows)), errors[method], 1e-10);
            } finally { y.delete(); }
        } finally { g.delete(); }
    }
    return errors;
}
function cubicTemporal(p, h) {
    const c = {...config, columns: 1024, spacingX: .5, spacingY: 1};
    const s = state(c.columns * c.rows), b = 1e-7, u = 1, t = .4;
    for (let i = 0; i < c.columns; ++i) {
        const x = (i + .5) * c.spacingX - 256, z = Math.max(-100, Math.min(100, x)) + 100;
        const rho = 1 + b * (z ** 3 + (Math.abs(x) < 100 ? .25 * c.spacingX ** 2 * z : 0));
        for (let j = 0; j < c.rows; ++j) put(s, i + c.columns * j, conserved(rho, u, 0, 1));
    }
    const g = new p.PeriodicEulerGasGrid(c);
    try {
        g.setState(s); const d = g.stepSecondOrder(t, {...options, maxSubstep: h});
        audit(s, g.state(), d, c);
        assert.equal(d.rejectedAttempts, 0); assert.equal(d.positivityLimitedCells, 0);
        assert.ok(4 * d.substeps < 190, 'polynomial patch must contain the full RK2 stencil');
        const z = (512 + .5) * c.spacingX - 256 - u * t + 100;
        const exact = 1 + b * (z ** 3 + .25 * c.spacingX ** 2 * z);
        const spatial = .5 * b * u * c.spacingX ** 2 * t;
        const temporal = g.state().density[512] - exact - spatial;
        // Fixed h except a possible final remainder: sum(h_i^3) is explicit.
        const remainder = t - (d.substeps - 1) * h;
        const expected = b * u ** 3 * ((d.substeps - 1) * h ** 3 + remainder ** 3);
        assert.ok(Math.abs(temporal - expected) < 1e-14);
        return {temporal: Math.abs(temporal), spatial, continuum: g.state().density[512] - exact};
    } finally { g.delete(); }
}
function nonlinearTemporal(p) {
    const c = {...config, columns: 64, spacingX: 1 / 64, spacingY: .5}, duration = .01;
    const s = waveState(c, 0, .123), reference = rk4(s, c, duration, 1024);
    const refined = rk4(s, c, duration, 2048), errors = [];
    for (const h of [.0025, .00125, .000625]) {
        const g = new p.PeriodicEulerGasGrid(c);
        try {
            g.setState(s); const d = g.stepSecondOrder(duration, {...options, maxSubstep: h});
            audit(s, g.state(), d, c);
            assert.equal(d.rejectedAttempts, 0); assert.equal(d.positivityLimitedCells, 0);
            errors.push(l1(g.state(), refined));
        } finally { g.delete(); }
    }
    assert.ok(l1(reference, refined) < errors[2] / 100, 'independent RK4 refinement must resolve temporal errors');
    for (let i = 1; i < errors.length; ++i) assert.ok(errors[i - 1] / errors[i] > 3.5 && errors[i - 1] / errors[i] < 4.5);
    const spatial = l1(refined, waveState(c, duration, .123));
    assert.ok(spatial > errors[0]);
    return {errors, spatial, referenceDifference: l1(reference, refined)};
}

function retryFixture(p) {
    const c = {...config, columns: 64, spacingX: 2 / 64, spacingY: .5};
    const s = state(c.columns * c.rows);
    for (let k = 0; k < s.density.length; ++k)
        put(s, k, k % c.columns < 32 ? conserved(1, 0, 0, 1) : conserved(.125, 0, 0, .1));
    const g = new p.PeriodicEulerGasGrid(c), o = {...options, cflSafety: .99};
    let d, expected;
    try {
        g.setState(s); d = g.stepSecondOrder(.12, o); expected = g.state();
        audit(s, expected, d, c);
        assert.ok(d.rejectedAttempts > 0); assert.ok(d.maximumRejectedCfl > o.cflSafety / 2);
        assert.deepEqual([d.attempts, d.substeps, d.rejectedAttempts, d.cellVisits], [53, 29, 24, 86912]);
    } finally { g.delete(); }
    const exactOptions = {...o, maximumCellVisits: d.cellVisits, maximumAttempts: d.attempts, maximumSubsteps: d.substeps};
    const replay = new p.PeriodicEulerGasGrid(c);
    try {
        replay.setState(s); assert.deepEqual(replay.stepSecondOrder(.12, exactOptions), d);
        assert.deepEqual(replay.state(), expected);
    } finally { replay.delete(); }
    for (const [f, value] of [['maximumCellVisits', d.cellVisits - 1], ['maximumAttempts', d.attempts - 1],
        ['maximumSubsteps', d.substeps - 1], ['maximumRetriesPerSubstep', 0]]) {
        const rejected = new p.PeriodicEulerGasGrid(c);
        try {
            rejected.setState(s); rejected.stepSecondOrder(0);
            const before = snapshot(rejected);
            assert.throws(() => rejected.stepSecondOrder(.12, {...exactOptions, [f]: value}));
            assert.deepEqual(snapshot(rejected), before, `${f} must roll back all earlier accepted scratch substeps`);
        } finally { rejected.delete(); }
    }
    return d;
}
function physicalSmoke(p) {
    const defaults = new p.PeriodicEulerGasGrid();
    try {
        const d = defaults.lastSecondOrderStep();
        assert.deepEqual(Object.keys(d).sort(), [...baseFields, ...extended].sort());
        for (const f of baseFields.concat(extended).filter(f => !['initial', 'final'].includes(f)))
            assert.equal(d[f], f === 'minimumSlopeScale' ? 1 : f === 'zeroDurationNoOp' ? false : 0);
        summaryFields.forEach(f => { assert.equal(d.initial[f], 0); assert.equal(d.final[f], 0); });
        assert.deepEqual(defaults.lastStep(), base(d));
    } finally { defaults.delete(); }
    for (const [nx, ny] of [[7, 5], [2, 5], [5, 2], [2, 2]]) {
        const c = {...config, columns: nx, rows: ny, spacingX: .7, spacingY: 1.3};
        const s = state(nx * ny);
        for (let k = 0; k < nx * ny; ++k)
            put(s, k, conserved(1 + .1 * Math.sin(k), .2 * Math.cos(k), .15 * Math.sin(2 * k), 1 + .1 * Math.cos(3 * k)));
        const expected = rk2(s, c, .001), g = new p.PeriodicEulerGasGrid(c);
        const rotated = new p.PeriodicEulerGasGrid({...c, columns: ny, rows: nx, spacingX: c.spacingY, spacingY: c.spacingX});
        const replay = new p.PeriodicEulerGasGrid(c);
        try {
            g.setState(s); const d = g.stepSecondOrder(.001), actual = g.state();
            assert.equal(d.substeps, 1); assert.equal(d.positivityLimitedCells, 0);
            audit(s, actual, d, c); closeState(actual, expected);
            rotated.setState(rotate(s, nx, ny)); rotated.stepSecondOrder(.001);
            closeState(rotated.state(), rotate(actual, nx, ny));
            replay.setState(s); assert.deepEqual(replay.stepSecondOrder(.001), d);
            assert.deepEqual(replay.state(), actual);
        } finally { g.delete(); rotated.delete(); replay.delete(); }
    }
    const ec = {...config, columns: 33, spacingX: 1 / 33, spacingY: .5}, es = state(66);
    for (let k = 0; k < 66; ++k) put(es, k, conserved(1 + .2 * Math.cos(2 * Math.PI * (k % 33) / 33), .7, 0, 1));
    assert.equal(mc(es.density[0] - es.density[32], es.density[1] - es.density[0]), 0);
    const ef = physicalFaces(es, ec);
    assert.equal(ef.faces[0][0][0], es.density[0]); assert.equal(ef.faces[0][1][0], es.density[0]);
    const extrema = new p.PeriodicEulerGasGrid(ec);
    try {
        extrema.setState(es); const d = extrema.stepSecondOrder(.0001);
        assert.ok(d.limitedSlopeCells > 0); assert.equal(d.positivityLimitedCells, 0);
        assert.ok(extrema.state().density[0] < es.density[0]);
        closeState(extrema.state(), rk2(es, ec, .0001));
        audit(es, extrema.state(), d, ec);
    } finally { extrema.delete(); }
    const uniform = new p.PeriodicEulerGasGrid(config), s = uniform.state();
    for (let k = 0; k < 4; ++k) put(s, k, conserved(2, -.7, .2, 3));
    let owned;
    try {
        uniform.setState(s);
        const d = uniform.stepSecondOrder(.01, {...options, maxSubstep: .01, maximumSubsteps: 1,
            maximumAttempts: 1, maximumRetriesPerSubstep: 0, maximumCellVisits: 76});
        audit(s, uniform.state(), d, config); assert.deepEqual(uniform.state(), s);
        assert.equal(d.cellVisits, 76); assert.equal(d.reconstructionTrials, 8);
        assert.deepEqual(uniform.lastStep(), base(d)); assert.deepEqual(uniform.lastSecondOrderStep(), d);
        owned = snapshot(uniform);
        const changed = copy(s); changed.density.fill(2.1);
        uniform.setState(changed);
        assert.equal(uniform.time(), .01); assert.deepEqual(uniform.lastSecondOrderStep(), d);
        uniform.step(.01); assert.deepEqual(uniform.lastSecondOrderStep(), d);
        assert.equal(uniform.time(), .02);
        const zero = uniform.stepSecondOrder(0, {...options, maximumSubsteps: 0, maximumAttempts: 0,
            maximumRetriesPerSubstep: 0, maximumCellVisits: 12});
        audit(changed, uniform.state(), zero, config);
        assert.equal(zero.timeBefore, .02); assert.equal(zero.timeAfter, .02);
        assert.deepEqual(uniform.lastStep(), base(zero)); assert.deepEqual(uniform.lastSecondOrderStep(), zero);
        const fresh = snapshot(uniform), a = uniform.lastStep(), b = uniform.lastSecondOrderStep();
        a.initial.mass = a.final.mass = 999; b.initial.mass = b.final.mass = 999;
        b.minimumSlopeScale = 999; zero.initial.mass = zero.final.mass = 999;
        assert.deepEqual(snapshot(uniform), fresh);
        const maxima = {maximumSubsteps: 1000000, maximumCellVisits: 1000000000,
            maximumAttempts: 1000000, maximumRetriesPerSubstep: 64};
        assert.deepEqual(uniform.stepSecondOrder(0, {...options, ...maxima}), fresh.lastSecondOrderStep);
        for (const f of Object.keys(options)) {
            const incomplete = {...options}; delete incomplete[f];
            assert.throws(() => uniform.stepSecondOrder(.01, incomplete));
            const bad = f.startsWith('maximum') ? [-1, .5, NaN, Infinity, -Infinity, 2 ** 32, 2 ** 32 + 1, maxima[f] + 1] : [0, -1, NaN, Infinity, -Infinity];
            for (const value of bad) {
                assert.throws(() => uniform.stepSecondOrder(.01, {...options, [f]: value}));
                assert.deepEqual(snapshot(uniform), fresh);
            }
        }
        for (const fail of [() => uniform.stepSecondOrder(-1), () => uniform.stepSecondOrder(NaN),
            () => uniform.stepSecondOrder(Infinity), () => uniform.stepSecondOrder(.01, {...options, cflSafety: 1}),
            () => uniform.stepSecondOrder(.01, {...options, maximumCellVisits: 75}),
            () => uniform.stepSecondOrder(.01, {...options, maximumAttempts: 0}),
            () => uniform.stepSecondOrder(.01, {...options, maximumSubsteps: 0}),
            () => uniform.stepSecondOrder(0, {...options, maximumCellVisits: 11})]) {
            assert.throws(fail); assert.deepEqual(snapshot(uniform), fresh);
        }
    } finally { uniform.delete(); }
    assert.equal(owned.state.density[0], 2); assert.equal(owned.lastSecondOrderStep.cellVisits, 76);
    const contacts = [32, 64, 128].map(n => contact(p, n));
    const waves = [32, 64, 128].map(n => nonlinearSpatial(p, n));
    assert.ok(contacts[1].stepSecondOrder / contacts[2].stepSecondOrder > 3);
    assert.ok(contacts[0].stepSecondOrder / contacts[1].stepSecondOrder > 3);
    assert.ok(contacts[2].stepSecondOrder < contacts[2].step / 5);
    assert.ok(waves[0].stepSecondOrder / waves[1].stepSecondOrder > 3.2);
    assert.ok(waves[1].stepSecondOrder / waves[2].stepSecondOrder > 3.2);
    assert.ok(waves[2].stepSecondOrder < waves[2].step / 5);
    const cubic = [.05, .025, .0125].map(h => cubicTemporal(p, h));
    for (let i = 1; i < cubic.length; ++i)
        assert.ok(cubic[i - 1].temporal / cubic[i].temporal > 3.9 && cubic[i - 1].temporal / cubic[i].temporal < 4.1);
    const nonlinear = nonlinearTemporal(p), retry = retryFixture(p);
    for (const gamma of [1.01, 1.4, 3, 20]) for (const scale of [1e-150, 1, 1e150]) {
        const c = {...config, columns: 8, rows: 6, spacingX: .4, spacingY: .7, gamma};
        const gas = new p.PeriodicEulerGasGrid(c), cold = gas.state();
        try {
            for (let j = 0; j < c.rows; ++j) for (let i = 0; i < c.columns; ++i) {
                const x = 2 * Math.PI * (i + .5) / c.columns, y = 2 * Math.PI * (j + .5) / c.rows;
                put(cold, i + c.columns * j, conserved(scale * (1 + .5 * Math.cos(x + y)),
                    .8 * Math.sin(x), -.6 * Math.cos(y), 1e-12 * scale, gamma));
            }
            gas.setState(cold); const d = gas.stepSecondOrder(.4, {...options, cflSafety: .99});
            audit(cold, gas.state(), d, c);
            assert.ok(d.minimumSlopeScale < 1 && d.positivityLimitedCells > 0);
            if (gamma >= 1.4) assert.ok(d.zeroSlopeFallbackCells > 0);
        } finally { gas.delete(); }
    }
    for (const gamma of [1 + Number.EPSILON, 1.01, 3, 20, 1e150]) {
        const c = {...config, gamma}, gas = new p.PeriodicEulerGasGrid(c), s = gas.state();
        try {
            const q = gamma > 1e100 ? conserved(1, 1e-150, 0, 1e-150, gamma) : conserved(1, .3, -.2, gamma - 1, gamma);
            for (let k = 0; k < 4; ++k) put(s, k, q);
            gas.setState(s); const d = gas.stepSecondOrder(.001);
            assert.deepEqual(gas.state(), s); audit(s, gas.state(), d, c);
        } finally { gas.delete(); }
    }
    const huge = new p.PeriodicEulerGasGrid({...config, columns: 4, spacingX: 1e-154, spacingY: 1e-154});
    try {
        const hs = huge.state();
        for (let k = 0; k < 8; ++k) put(hs, k, [1.5e308, k % 4 < 2 ? .75e308 : -.75e308, 0, 1.7e308]);
        huge.setState(hs); const before = snapshot(huge);
        assert.throws(() => huge.stepSecondOrder(4e-155)); assert.deepEqual(snapshot(huge), before);
    } finally { huge.delete(); }
    const clock = new p.PeriodicEulerGasGrid({...config, spacingX: 1e154, spacingY: 1e154});
    try {
        const cs = clock.state(); for (let k = 0; k < 4; ++k) put(cs, k, [.1, 0, 0, 1e-320]);
        clock.setState(cs); clock.stepSecondOrder(1e308, {...options, maxSubstep: 1e308});
        assert.deepEqual(clock.state(), cs); const before = snapshot(clock);
        for (const duration of [1, 1e308]) {
            assert.throws(() => clock.stepSecondOrder(duration, {...options, maxSubstep: 1e308}));
            assert.deepEqual(snapshot(clock), before);
        }
    } finally { clock.delete(); }
    console.log(`PASS: browser Euler MC/SSPRK2 physical axes/two-cell/extrema, 32/64/128 contact ${JSON.stringify(contacts)}, nonlinear spatial ${JSON.stringify(waves)}; padded cubic ${JSON.stringify(cubic)}; independent nonlinear RK4 temporal ${JSON.stringify(nonlinear)}; stage retries ${retry.rejectedAttempts}, exact visits ${retry.cellVisits}; cold positivity/fallback, budgets/late rollback/history/ownership/clock/range`);
}

function smoke(p) {
    const stats = typeof p.boundaryTestStats === 'function' ?
        () => ({...p.boundaryTestStats(), emvalHandles: p.count_emval_handles()}) : null;
    const before = stats && stats(), stack = stats && p._emscripten_stack_get_current();
    physicalSmoke(p);
    if (stats) {
        assert.deepEqual(stats(), before);
        assert.equal(p._emscripten_stack_get_current(), stack);
        assert.equal(before.emvalHandles, 0); assert.equal(before.uncaught, 0);
        console.log(`PASS: Euler second-order physical smoke exact stack=${stack}, heap=${before.heap}, uncaught=0, emval=0`);
    }
}

function stress(p, probe) {
    const g = new p.PeriodicEulerGasGrid(config), source = g.state();
    g.stepSecondOrder(0); const saved = snapshot(g), foreign = new Error('second-order foreign conversion');
    const stack = () => p._emscripten_stack_get_current();
    const stats = () => ({...p.boundaryTestStats(), emvalHandles: p.count_emval_handles()});
    const nativeError = e => e instanceof Error && !('excPtr' in e);
    const failures = [
        () => g.stepSecondOrder(-1), () => g.stepSecondOrder(NaN), () => g.stepSecondOrder(Infinity),
        () => g.stepSecondOrder(.01, {...options, maximumSubsteps: 0}),
        () => g.stepSecondOrder(.01, {...options, maximumAttempts: 0}),
        () => g.stepSecondOrder(.02, {...options, maxSubstep: .01, maximumCellVisits: 76}),
        () => g.stepSecondOrder(0, {...options, maximumCellVisits: 11}),
        () => g.setState({...source, totalEnergy: [1, 1, 1, 0]})
    ].map(fn => [fn, nativeError]);
    for (const f of Object.keys(options)) {
        const missing = {...options}; delete missing[f];
        failures.push([() => g.stepSecondOrder(.01, missing), nativeError]);
        failures.push([() => g.stepSecondOrder(.01, {...options, get [f]() { throw foreign; }}), e => e === foreign]);
        failures.push([() => g.stepSecondOrder(.01, {...options, get [f]() { g.stepSecondOrder(-1); }}), nativeError]);
        if (f.startsWith('maximum'))
            failures.push([() => g.stepSecondOrder(.01, {...options, [f]: 2 ** 32 + 1}), nativeError]);
    }
    for (const f of fields) {
        failures.push([() => g.setState({...source, get [f]() { throw foreign; }}), e => e === foreign]);
        failures.push([() => {
            const a = source[f].slice(); Object.defineProperty(a, 3, {get() { throw foreign; }});
            g.setState({...source, [f]: a});
        }, e => e === foreign]);
    }
    failures.push([() => g.stepSecondOrder(.01, new Proxy(options, {get(t, k) {
        if (k === 'maximumRetriesPerSubstep') throw foreign; return t[k];
    }})), e => e === foreign]);
    for (const value of [undefined, null, 73, 'foreign primitive'])
        failures.push([() => g.stepSecondOrder(.01, {...options, get maximumRetriesPerSubstep() { throw value; }}), e => e === value]);
    let total = 0;
    function batch() {
        for (const [call, check] of failures) {
            const before = stack(); assert.throws(call, check); assert.equal(stack(), before); ++total;
        }
        for (const f of ['duration', ...Object.keys(options)]) {
            let conversions = 0;
            const scalar = {valueOf() { ++conversions; throw foreign; }};
            assert.throws(() => g.stepSecondOrder(f === 'duration' ? scalar : .01,
                f === 'duration' ? options : {...options, [f]: scalar}), e =>
                conversions ? e === foreign : e instanceof TypeError);
            assert.ok(conversions <= 1);
        }
        const reentrant = {...options, get maximumRetriesPerSubstep() {
            assert.equal(probe.method(), 7); g.stepSecondOrder(0); return 16;
        }};
        g.stepSecondOrder(0, reentrant);
        const alias = g.clone();
        assert.deepEqual(alias.lastSecondOrderStep(), saved.lastSecondOrderStep);
        alias.delete();
        // All direct scalar converters and both getter/coercion entry points.
        // ASSERTIONS builds may reject an object before invoking valueOf.
        for (const callback of ['duration', ...Object.keys(options)]) for (const kind of ['getter', 'valueOf']) {
            const receiver = new p.PeriodicEulerGasGrid(config), retained = receiver.clone();
            receiver.stepSecondOrder(0); const before = snapshot(retained); let coercions = 0;
            const convert = () => { ++coercions; receiver.delete(); return callback === 'duration' ? .01 : options[callback]; };
            let duration = .01, opt = {...options};
            if (callback === 'duration') {
                duration = kind === 'getter' ? {get valueOf() { convert(); return () => .01; }} : {valueOf: convert};
            } else if (kind === 'getter') Object.defineProperty(opt, callback, {get: convert});
            else opt[callback] = {valueOf: convert};
            try {
                assert.throws(() => receiver.stepSecondOrder(duration, opt), e =>
                    coercions ? /deleted/i.test(e.message) : e instanceof TypeError);
                assert.ok(coercions <= 1);
                assert.equal(receiver.isDeleted(), coercions === 1);
                assert.deepEqual(snapshot(retained), before);
            } finally {
                if (!receiver.isDeleted()) receiver.delete(); retained.delete();
            }
        }
        // Conversion can retain aliases or delete then throw. The original JS
        // exception remains the same object, with the native instance unchanged.
        const receiver = new p.PeriodicEulerGasGrid(config), retained = receiver.clone();
        receiver.stepSecondOrder(0); const before = snapshot(retained);
        try {
            assert.throws(() => receiver.stepSecondOrder(.01, {...options,
                get maximumRetriesPerSubstep() { receiver.delete(); throw foreign; }}), e => e === foreign);
            assert.deepEqual(snapshot(retained), before);
            assert.deepEqual(retained.stepSecondOrder(0), before.lastSecondOrderStep);
        } finally { if (!receiver.isDeleted()) receiver.delete(); retained.delete(); }
    }
    try {
        for (let i = 0; i < 20; ++i) batch();
        total = 0; const before = stats(), savedStack = stack();
        assert.equal(before.emvalHandles, 0); assert.equal(before.uncaught, 0);
        for (let i = 0; i < 1000; ++i) batch();
        assert.deepEqual(stats(), before); assert.equal(stack(), savedStack);
        assert.deepEqual(snapshot(g), saved);
        console.log(`PASS: browser Euler second-order boundary stress; 1000 batches/${total} listed failures plus all duration/option getters/valueOf deletion, nested native/foreign failures, reentrancy/retained aliases/history; exact stack=${savedStack}, heap=${before.heap}, uncaught=${before.uncaught}, emval=0`);
    } finally { g.delete(); }
}
module.exports = {smoke, stress};
