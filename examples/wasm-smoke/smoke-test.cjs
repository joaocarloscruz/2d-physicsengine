const assert = require("node:assert/strict");
const createPhysicsEngineModule = require("./physics_engine.js");

function testQueries(physics) {
    const engine = new physics.Engine();
    const material = {density: 1, restitution: 0, staticFriction: 0, dynamicFriction: 0};
    const shape = new physics.Circle(1);
    const first = physics.createRigidBody(shape, material, {x: 2, y: 0}, true);
    const second = physics.createRigidBody(shape, material, {x: 2, y: 0}, false);
    shape.delete();
    // Reversed registration must not change stable-ID ties.
    engine.addBody(second); engine.addBody(first);
    const firstId = first.getId(), secondId = second.getId();
    assert.equal(typeof firstId, "bigint"); assert.ok(firstId < secondId);
    const start = {x: 0, y: 0}, end = {x: 4, y: 0}, center = {x: 2, y: 0};
    const all = [physics.queryPoint(engine, center), physics.queryCircle(engine, {x: 3.5, y: 0}, 0.5),
        physics.rayCastAll(engine, start, end), physics.sweepCircleAll(engine, start, end, 0.5)];
    for (const results of all) {
        assert.equal(results.size(), 2);
        assert.equal(results.getBodyId(0), firstId); assert.equal(results.getBodyId(1), secondId);
        for (const bad of [-1, 0.5, NaN, Infinity, 2, 2**32, 2**32 + 1]) {
            assert.throws(() => results.getBodyId(bad)); assert.throws(() => results.getBody(bad));
            if (results.getHit) assert.throws(() => results.getHit(bad));
        }
    }
    assert.deepEqual(all[2].getHit(0), {fraction: 0.25, point: {x: 1, y: 0}, normal: {x: -1, y: 0}});
    assert.deepEqual(all[3].getHit(0), {fraction: 0.125, center: {x: 0.5, y: 0}, contactPoint: {x: 1, y: 0}, normal: {x: -1, y: 0}});
    const nearestRay = physics.rayCastNearest(engine, start, end), nearestSweep = physics.sweepCircleNearest(engine, start, end, 0.5);
    for (const results of [nearestRay, nearestSweep]) {
        assert.equal(results.size(), 1); assert.equal(results.getBodyId(0), firstId); results.delete();
    }
    const calls = [f => physics.queryPoint(engine, center, f), f => physics.queryCircle(engine, center, 0.1, f),
        f => physics.rayCastAll(engine, start, end, f), f => physics.rayCastNearest(engine, start, end, f),
        f => physics.sweepCircleAll(engine, start, end, 0.5, f), f => physics.sweepCircleNearest(engine, start, end, 0.5, f)];
    first.setCollisionCategoryBits(2); first.setCollisionMaskBits(4);
    second.setCollisionCategoryBits(4); second.setCollisionMaskBits(2);
    for (const call of calls) {
        const matched = call({categoryBits: 4, maskBits: 2});
        assert.equal(matched.size(), 1); assert.equal(matched.getBodyId(0), firstId); matched.delete();
        for (const filter of [{categoryBits: 0, maskBits: 0xffffffff}, {categoryBits: 2, maskBits: 2}]) {
            const empty = call(filter); assert.equal(empty.size(), 0); empty.delete();
        }
        for (const field of ["categoryBits", "maskBits"]) for (const bad of [-1, 0.5, NaN, Infinity, 2**32, 2**32 + 2])
            assert.throws(() => call({categoryBits: 0xffffffff, maskBits: 0xffffffff, [field]: bad}));
    }
    const rayCopy = all[2].getHit(0); rayCopy.point.x = 99; rayCopy.normal.x = 99;
    const sweepCopy = all[3].getHit(0); sweepCopy.center.x = 99;
    first.setPosition({x: 20, y: 0});
    engine.clearBodies(); first.delete(); second.delete(); engine.delete();
    assert.equal(all[2].getHit(0).point.x, 1); assert.equal(all[3].getHit(0).center.x, 0.5);
    const retained = all[0].getBody(0);
    for (const results of all) { assert.equal(results.getBodyId(0), firstId); results.delete(); }
    assert.deepEqual(retained.getPosition(), {x: 20, y: 0}); retained.delete();

    const polygonEngine = new physics.Engine(), box = physics.Polygon.makeBox(2, 2);
    const polygon = physics.createRigidBody(box, material, {x: 0, y: 0}, true);
    box.delete(); polygonEngine.addBody(polygon); polygon.delete();
    const a = {x: 2, y: 2}, b = {x: 1.8, y: 1.8};
    const cornerMiss = physics.sweepCircleNearest(polygonEngine, a, b, 1);
    assert.equal(cornerMiss.size(), 0); cornerMiss.delete();
    const cornerHit = physics.sweepCircleNearest(polygonEngine, a, {x: 0, y: 0}, 1);
    const hit = cornerHit.getHit(0);
    assert.ok(Math.abs(hit.fraction - (1 - 1/Math.sqrt(2))/2) < 1e-12);
    assert.deepEqual(hit.contactPoint, {x: 1, y: 1});
    assert.ok(Math.abs(hit.normal.x - 1/Math.sqrt(2)) < 1e-7); cornerHit.delete();
    polygonEngine.clearBodies();
    for (const result of [physics.queryPoint(polygonEngine, a), physics.queryCircle(polygonEngine, a, 0),
        physics.rayCastNearest(polygonEngine, a, b), physics.sweepCircleNearest(polygonEngine, a, b, 0)]) {
        assert.equal(result.size(), 0); assert.throws(() => result.getBody(0)); result.delete();
    }
    for (const bad of [-1, NaN, Infinity]) {
        assert.throws(() => physics.queryCircle(polygonEngine, a, bad));
        assert.throws(() => physics.sweepCircleAll(polygonEngine, a, b, bad));
    }
    assert.throws(() => physics.queryPoint(polygonEngine, {x: NaN, y: 0}));
    assert.throws(() => physics.rayCastAll(polygonEngine, a, {x: Infinity, y: 0}));
    polygonEngine.delete();
}

function testScalarTransport(physics) {
    const near=(a,b,t=4e-14)=>assert.ok(Math.abs(a-b)<=t, `${a} vs ${b}, tolerance ${t}`);
    const nearArray=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,k)=>near(v,b[k],t));};
    const options={cflSafety:.9,maxSubstep:.1,maximumSubsteps:10000,maximumCellVisits:100000000};
    const uniform=(n,u,v)=>({xFaces:Array(n).fill(u),yFaces:Array(n).fill(v)});
    const put=(grid,q,v)=>{grid.setState(q);grid.setVelocities(v.xFaces,v.yFaces);};
    // Independent cell donor matrix: each row uses four incoming neighbors,
    // rather than reproducing the native equal/opposite face accumulation.
    const donor=(q,v,c,h)=>q.map((value,k)=>{
        const i=k%c.columns,j=Math.floor(k/c.columns), nx=c.columns,ny=c.rows;
        const l=(i+nx-1)%nx+nx*j,r=(i+1)%nx+nx*j,b=i+nx*((j+ny-1)%ny),t=i+nx*((j+1)%ny);
        const out=(Math.max(v.xFaces[r],0)+Math.max(-v.xFaces[k],0))/c.spacingX+
            (Math.max(v.yFaces[t],0)+Math.max(-v.yFaces[k],0))/c.spacingY;
        return (1-h*out)*value+h*(Math.max(v.xFaces[k],0)*q[l]+Math.max(-v.xFaces[r],0)*q[r])/c.spacingX+
            h*(Math.max(v.yFaces[k],0)*q[b]+Math.max(-v.yFaces[t],0)*q[t])/c.spacingY;
    });
    const defaults=new physics.PeriodicScalarTransport(),base=defaults.getConfig();
    assert.deepEqual(base,{columns:16,rows:16,spacingX:1,spacingY:1});
    assert.deepEqual(defaults.getState(),Array(256).fill(0));
    assert.deepEqual(defaults.getVelocities(),uniform(256,0,0));assert.equal(defaults.getTime(),0);
    const diagnosticNames=['substeps','cellVisits','duration','timeBefore','timeAfter','lastSubstep',
        'maximumOutflowRate','outflowRateBound','maximumAbsDivergence','maximumCfl','initialIntegratedScalar',
        'finalIntegratedScalar','initialAbsoluteIntegral','finalAbsoluteIntegral','integratedScalarDrift',
        'conservationRoundoffAllowance','initialMinimum','initialMaximum','finalMinimum','finalMaximum',
        'rangeRoundoffAllowance','nonnegativeInput','discreteDivergenceFree','zeroDurationNoOp'];
    assert.deepEqual(Object.keys(defaults.getLastStep()).sort(),diagnosticNames.sort());
    for(const [name,value] of Object.entries(defaults.getLastStep()))
        assert.equal(value,['nonnegativeInput','discreteDivergenceFree','zeroDurationNoOp'].includes(name)?false:0);
    defaults.step(.01);assert.equal(defaults.getTime(),.01);defaults.delete();
    for(const [columns,rows] of [[7,5],[2,5],[5,2],[2,2]]) {
        const c={columns,rows,spacingX:.23,spacingY:.41},n=columns*rows,grid=new physics.PeriodicScalarTransport(c);
        const q=Array.from({length:n},(_,k)=>.2+Math.sin(.7*k));
        const v={xFaces:q.map((_,k)=>.3*Math.sin(.31*k+.7)),yFaces:q.map((_,k)=>-.2*Math.cos(.9*k))};
        put(grid,q,v);const d=grid.step(.02);
        assert.equal(d.substeps,1);nearArray(grid.getState(),donor(q,v,c,.02));
        near(d.finalIntegratedScalar,q.reduce((a,b)=>a+b)*c.spacingX*c.spacingY,3e-15);
        assert.ok(Math.abs(d.integratedScalarDrift)<=d.conservationRoundoffAllowance);
        assert.equal(d.cellVisits,11*n);assert.ok(d.maximumCfl<.9);
        grid.delete();
    }
    for(const [columns,rows] of [[13,11],[2,5],[5,2],[2,2]]) for(const u of [-.3,.3]) for(const v of [-.2,.2]) {
        const c={columns,rows,spacingX:.23,spacingY:.41},n=columns*rows,grid=new physics.PeriodicScalarTransport(c);
        const tx=2*Math.PI/columns,ty=2*Math.PI/rows;
        const q=Array.from({length:n},(_,k)=>2+.4*Math.cos(tx*(k%columns)+ty*Math.floor(k/columns)+.31));
        put(grid,q,uniform(n,u,v));const d=grid.step(.1,{...options,maxSubstep:.01});
        // Independent complex Fourier amplification using the actual accepted h.
        const cx=d.lastSubstep*Math.abs(u)/c.spacingX,cy=d.lastSubstep*Math.abs(v)/c.spacingY;
        const gr=1-cx-cy+cx*Math.cos(tx)+cy*Math.cos(ty);
        const gi=-cx*Math.sign(u)*Math.sin(tx)-cy*Math.sign(v)*Math.sin(ty);
        const amplitude=Math.hypot(gr,gi)**d.substeps,phase=d.substeps*Math.atan2(gi,gr);
        nearArray(grid.getState(),q.map((_,k)=>2+.4*amplitude*Math.cos(tx*(k%columns)+ty*Math.floor(k/columns)+.31+phase)));
        assert.ok(amplitude<1);assert.ok(d.discreteDivergenceFree);assert.ok(d.nonnegativeInput);grid.delete();
    }
    const c={columns:2,rows:2,spacingX:1,spacingY:1},grid=new physics.PeriodicScalarTransport(c);
    put(grid,[1,2,1,2],{xFaces:[1,-1,1,-1],yFaces:[0,0,0,0]});
    const two=grid.step(.1);nearArray(grid.getState(),[1.4,1.6,1.4,1.6],1e-15);
    assert.equal(two.maximumOutflowRate,2);assert.equal(two.maximumAbsDivergence,2);
    assert.equal(two.discreteDivergenceFree,false);near(two.finalIntegratedScalar,6,1e-15);
    put(grid,[1,1,1,1],{xFaces:[1,-1,1,-1],yFaces:[0,0,0,0]});
    const compression=grid.step(.1);nearArray(grid.getState(),[1.2,.8,1.2,.8],1e-15);
    assert.ok(compression.finalMaximum>1);assert.ok(compression.finalMinimum<1);near(compression.integratedScalarDrift,0);
    const snapshot=()=>({q:grid.getState(),v:grid.getVelocities(),d:grid.getLastStep(),time:grid.getTime()});
    const retained=grid.getLastStep(),q=[.731,-.137,1.123,3.331];
    put(grid,q,uniform(4,0,0));assert.deepEqual(grid.getLastStep(),retained);
    const noOp=grid.step(0,{...options,maximumSubsteps:0,maximumCellVisits:12});
    assert.equal(noOp.cellVisits,12);assert.equal(noOp.substeps,0);assert.equal(noOp.zeroDurationNoOp,true);
    assert.equal(noOp.timeBefore,noOp.timeAfter);assert.equal(noOp.duration,0);assert.equal(noOp.maximumCfl,0);
    assert.equal(noOp.integratedScalarDrift,0);assert.equal(noOp.conservationRoundoffAllowance,0);
    assert.equal(noOp.rangeRoundoffAllowance,0);assert.deepEqual(grid.getState(),q);
    near(noOp.initialIntegratedScalar,q.reduce((a,b)=>a+b),1e-15);
    near(noOp.initialAbsoluteIntegral,q.reduce((a,b)=>a+Math.abs(b)),1e-15);
    assert.equal(noOp.initialIntegratedScalar,noOp.finalIntegratedScalar);
    assert.equal(noOp.initialAbsoluteIntegral,noOp.finalAbsoluteIntegral);
    assert.equal(noOp.initialMinimum,Math.min(...q));assert.equal(noOp.finalMinimum,noOp.initialMinimum);
    assert.equal(noOp.initialMaximum,Math.max(...q));assert.equal(noOp.finalMaximum,noOp.initialMaximum);
    assert.equal(noOp.maximumOutflowRate,0);assert.equal(noOp.outflowRateBound,0);
    assert.equal(noOp.maximumAbsDivergence,0);assert.equal(noOp.discreteDivergenceFree,true);assert.equal(noOp.nonnegativeInput,false);
    const before=snapshot();
    for(const work of [0,11]) {assert.throws(()=>grid.step(0,{...options,maximumSubsteps:0,maximumCellVisits:work}));assert.deepEqual(snapshot(),before);}
    const exact=grid.step(.01,{...options,maxSubstep:.01,maximumSubsteps:1,maximumCellVisits:44});
    assert.equal(exact.substeps,1);assert.equal(exact.cellVisits,44);assert.deepEqual(grid.getState(),q);
    // Every rejection preserves fields, prescribed advector, clock and the full last operation.
    const rollback=snapshot(),reject=fn=>{assert.throws(fn);assert.deepEqual(snapshot(),rollback);};
    for(const duration of [-1,NaN,Infinity]) reject(()=>grid.step(duration));
    for(const field of ['maximumSubsteps','maximumCellVisits']) for(const bad of [-1,.5,NaN,Infinity,2**32,2**32+1])
        reject(()=>grid.step(0,{...options,[field]:bad}));
    reject(()=>grid.step(0,{...options,maximumSubsteps:1000001}));
    reject(()=>grid.step(0,{...options,maximumCellVisits:1000000001}));
    for(const safety of [0,1,-1,NaN,Infinity]) reject(()=>grid.step(0,{...options,cflSafety:safety}));
    for(const h of [0,-1,NaN,Infinity]) reject(()=>grid.step(0,{...options,maxSubstep:h}));
    reject(()=>grid.step(.02,{...options,maxSubstep:.01,maximumSubsteps:1}));
    reject(()=>grid.step(.01,{...options,maximumCellVisits:43}));
    for(const bad of [[],[1,2,3],new Float64Array(4),Array(4),[1,2,3,NaN],[1,2,3,Infinity],['1',2,3,4],[null,2,3,4],[{},2,3,4]]) {
        reject(()=>grid.setState(bad));reject(()=>grid.setVelocities(bad,[0,0,0,0]));
        reject(()=>grid.setVelocities([0,0,0,0],bad));
    }
    for(const field of ['columns','rows']) for(const bad of [0,1,-1,.5,NaN,Infinity,262145,2**32,2**32+2])
        assert.throws(()=>new physics.PeriodicScalarTransport({...base,[field]:bad}));
    assert.throws(()=>new physics.PeriodicScalarTransport({...base,columns:512,rows:513}));
    for(const field of ['spacingX','spacingY']) for(const bad of [0,-1,NaN,Infinity,Number.MAX_VALUE])
        assert.throws(()=>new physics.PeriodicScalarTransport({...base,[field]:bad}));
    assert.throws(()=>new physics.PeriodicScalarTransport({...c,spacingX:1e-200,spacingY:1e-200}));
    const largest=new physics.PeriodicScalarTransport({...c,columns:512,rows:512});
    assert.equal(largest.getState().length,262144);largest.delete();
    grid.step(0,{...options,maximumSubsteps:1000000,maximumCellVisits:1000000000}); // inclusive caps
    put(grid,[1,0,0,0],uniform(4,.7,.3));const rate=grid.step(0).outflowRateBound;
    // Chosen duration is above the conservative CFL endpoint: one-step budget must reject.
    const cflBefore=snapshot();assert.throws(()=>grid.step(.9/rate,{...options,maxSubstep:2,maximumSubsteps:1}));
    assert.deepEqual(snapshot(),cflBefore);
    const bounded=grid.step(.9/rate,{...options,maxSubstep:2});assert.equal(bounded.substeps,2);assert.ok(bounded.maximumCfl<=.9);
    // This initial summary is finite. Concentration overflows only after staged donor transfers.
    const late=new physics.PeriodicScalarTransport({...c,spacingX:1e-154,spacingY:1e-154});
    put(late,[1e308,1e308,1e308,1e308],{xFaces:[1e-154,-1e-154,1e-154,-1e-154],yFaces:[0,0,0,0]});
    const lateBefore={q:late.getState(),v:late.getVelocities(),d:late.getLastStep()};
    assert.throws(()=>late.step(.4,{...options,maxSubstep:1}));
    assert.deepEqual({q:late.getState(),v:late.getVelocities(),d:late.getLastStep()},lateBefore);assert.equal(late.getTime(),0);late.delete();
    const tiny=new physics.PeriodicScalarTransport(c);put(tiny,[1e308,0,0,0],uniform(4,1e-308,0));
    assert.throws(()=>tiny.step(1e-20));assert.deepEqual(tiny.getState(),[1e308,0,0,0]);tiny.delete();
    for(const scale of [1e-200,1e200]) {
        const g=new physics.PeriodicScalarTransport({...c,spacingX:1e100,spacingY:1e-100});
        const reduced=[1,.5,.2,.7],v=uniform(4,.3e100,-.2e-100);
        put(g,reduced.map(value=>value*scale),v);const d=g.step(.1);
        nearArray(g.getState().map(value=>value/scale),donor(reduced,uniform(4,.3,-.2),c,.1),3e-15);
        assert.ok(Math.abs(d.integratedScalarDrift)<=d.conservationRoundoffAllowance);g.delete();
    }
    const pulseConfig={columns:32,rows:16,spacingX:.03,spacingY:.07},pulse=new physics.PeriodicScalarTransport(pulseConfig);
    const pulseQ=Array.from({length:512},(_,k)=>((k%32>=29||k%32<3)&&Math.floor(k/32)>=6&&Math.floor(k/32)<10)?1:0);
    const pulseV={xFaces:pulseQ.map((_,k)=>.3+.1*Math.sin(2*Math.PI*Math.floor(k/32)/16)),
        yFaces:pulseQ.map((_,k)=>-.2+.1*Math.cos(2*Math.PI*(k%32)/32))};
    put(pulse,pulseQ,pulseV);const pd=pulse.step(.6);assert.ok(pd.discreteDivergenceFree);assert.ok(pd.nonnegativeInput);
    pulse.getState().forEach(value=>{assert.ok(value>=0&&value<=1+2e-14);});near(pd.finalIntegratedScalar,24*.03*.07,2e-15);
    put(pulse,Array(512).fill(.7),pulseV);pulse.step(.6);nearArray(pulse.getState(),Array(512).fill(.7),3e-15);pulse.delete();
    const sinc=x=>Math.sin(x)/x,spatialError=nx=>{
        const c={columns:nx,rows:nx/2,spacingX:1/nx,spacingY:2/nx},n=c.columns*c.rows,g=new physics.PeriodicScalarTransport(c);
        const factor=sinc(Math.PI*c.spacingX)*sinc(2*Math.PI*c.spacingY),duration=.4;
        const phase=k=>2*Math.PI*((k%nx+.5)*c.spacingX+2*(Math.floor(k/nx)+.5)*c.spacingY);
        put(g,Array.from({length:n},(_,k)=>1+.3*factor*Math.cos(phase(k))),uniform(n,.7,-.2));
        g.step(duration,{...options,maxSubstep:duration/nx});
        const error=Math.sqrt(g.getState().reduce((sum,value,k)=>sum+(value-1-.3*factor*Math.cos(phase(k)-2*Math.PI*.3*duration))**2,0)/n);
        g.delete();return error;
    };
    const errors=[32,64,128,256].map(spatialError);
    assert.ok(errors[0]/errors[1]>1.6);assert.ok(errors[1]/errors[2]>1.75);assert.ok(errors[2]/errors[3]>1.85&&errors[2]/errors[3]<2.1);
    const tc={columns:32,rows:2,spacingX:1/32,spacingY:.5},theta=2*Math.PI/32,T=.4;
    const er=Math.exp(T/tc.spacingX*(Math.cos(theta)-1)),ei=-T/tc.spacingX*Math.sin(theta);
    let previous=0;
    for(const count of [16,32,64,128]) {
        const g=new physics.PeriodicScalarTransport(tc);put(g,Array.from({length:64},(_,k)=>1+.3*Math.cos(theta*(k%32))),uniform(64,1,0));
        g.step(T,{...options,maxSubstep:T/count});
        const error=Math.sqrt(g.getState().reduce((sum,value,k)=>sum+(value-1-.3*er*Math.cos(theta*(k%32)+ei))**2,0)/64);
        if(previous) assert.ok(previous/error>1.8&&previous/error<2.4);previous=error;g.delete();
    }
    const replay=new physics.PeriodicScalarTransport(c),initial=grid.getState(),advector=grid.getVelocities();
    put(replay,initial,advector);const originTime=grid.getTime();
    for(let i=0;i<12;++i) {grid.step(.03);replay.step(.03);assert.deepEqual(grid.getState(),replay.getState());}
    near(grid.getTime()-originTime,replay.getTime(),1e-15);replay.delete();
    const copiedInputs=grid.getState(),copiedV=grid.getVelocities();put(grid,copiedInputs,copiedV);
    const isolated=snapshot(),copied=snapshot();copied.q[0]=99;copied.v.xFaces[0]=99;copied.d.timeAfter=99;
    copiedInputs[0]=99;copiedV.yFaces[0]=99;const configCopy=grid.getConfig();configCopy.columns=999;
    assert.deepEqual(snapshot(),isolated);assert.deepEqual(grid.getConfig(),c);
    const retainedText=JSON.stringify(isolated);grid.delete();assert.equal(JSON.stringify(isolated),retainedText);
    assert.ok(Array.isArray(isolated.q));assert.ok(Number.isFinite(isolated.d.timeAfter));
    console.log(`PASS: scalar transport donor/Fourier/compression, positivity/mass, temporal/spatial refinement (${errors.join(', ')}), work/range rollback and copied lifetimes`);
}

function testMaxwell(physics) {
    const defaults = new physics.MaxwellGrid(), base = defaults.getConfig();
    assert.deepEqual(base, {columns:16, rows:16, spacingX:1, spacingY:1, permittivity:1,
        permeability:1, cflSafety:.9, maxSubstep:.1, maximumSubsteps:10000, maximumCellVisits:100000000});
    const near = (actual, expected, tolerance = 3e-12) => assert.ok(Math.abs(actual-expected) <= tolerance,
        `${actual} versus ${expected}, tolerance ${tolerance}`);
    const mode = (c, mx, my, e, b) => {
        const ax = 2*Math.sin(Math.PI*mx/c.columns)/c.spacingX;
        const ay = 2*Math.sin(Math.PI*my/c.rows)/c.spacingY;
        const fields = {ez:[], hx:[], hy:[]};
        for (let j=0; j<c.rows; ++j) for (let i=0; i<c.columns; ++i) {
            const theta = 2*Math.PI*(mx*i/c.columns+my*j/c.rows)+.31;
            fields.ez.push(e*Math.cos(theta));
            fields.hx.push(ay*b*Math.sin(theta+Math.PI*my/c.rows));
            fields.hy.push(-ax*b*Math.sin(theta+Math.PI*mx/c.columns));
        }
        return fields;
    };
    const sameFields = (a,b,tol=3e-12) => {
        for (const field of ['ez','hx','hy']) {
            assert.ok(Array.isArray(a[field])); assert.equal(a[field].length,b[field].length);
            for (let i=0; i<a[field].length; ++i) near(a[field][i],b[field][i],tol);
        }
    };
    // Independent spectral amplitude map, including staggered polarization.
    for (const [nx,ny] of [[12,10],[2,5],[5,2],[2,2]]) {
        const c = {...base, columns:nx, rows:ny, spacingX:.23, spacingY:.41,
            permittivity:2, permeability:3, maxSubstep:10};
        const grid = new physics.MaxwellGrid(c);
        near(grid.getWaveSpeed(),1/Math.sqrt(6),1e-16);
        const h = .7*grid.getStableTimeStep(), mx=1,my=1;
        const ax = 2*Math.sin(Math.PI/nx)/c.spacingX, ay = 2*Math.sin(Math.PI/ny)/c.spacingY;
        const g=ax*ax+ay*ay, z=h*Math.sqrt(g/6), d=1-z*z/2;
        let e=.7, b=-.13;
        grid.setState(mode(c,mx,my,e,b));
        for (let step=0; step<17; ++step) {
            const newE=d*e-h*g*b/c.permittivity;
            b=d*b+h*(1-z*z/4)*e/c.permeability; e=newE;
            grid.step(h);
        }
        sameFields(grid.getState(),mode(c,mx,my,e,b));
        const diag=grid.getDiagnostics();
        assert.equal(diag.lastSubsteps,1); assert.equal(diag.lastCellVisits,nx*ny*4);
        assert.equal(diag.lastSubstep,h); assert.equal(diag.modifiedEnergyStep,h);
        assert.equal(diag.stableTimeStep,grid.getStableTimeStep());
        near(diag.time,17*h); assert.ok(diag.maxAbsMagneticDivergence<2e-13);
        grid.delete();
    }
    // Traveling eigenmode: phase and synchronous magnetic polarization are
    // derived analytically from the spectral map, rather than iterated updates.
    const travelingConfig={...base,columns:12,rows:10,spacingX:.23,spacingY:.41,
        permittivity:2,permeability:3,maxSubstep:10};
    const traveling=new physics.MaxwellGrid(travelingConfig);
    const th=.8*traveling.getStableTimeStep(), steps=237;
    const tax=2*Math.sin(Math.PI/12)/.23, tay=2*Math.sin(Math.PI/10)/.41;
    const omega=Math.hypot(tax,tay)/Math.sqrt(6), z=th*omega;
    const magneticAmplitude=Math.sqrt(1-z*z/4)/(3*omega);
    const travelingFields={ez:[],hx:[],hy:[]}, expectedFields={ez:[],hx:[],hy:[]};
    const phase=steps*2*Math.asin(z/2);
    for (let j=0;j<10;++j) for (let i=0;i<12;++i) {
        const theta=2*Math.PI*(i/12+j/10)+.31;
        for (const [fields,advance] of [[travelingFields,0],[expectedFields,phase]]) {
            fields.ez.push(Math.cos(theta-advance));
            fields.hx.push(tay*magneticAmplitude*Math.cos(theta+Math.PI/10-advance));
            fields.hy.push(-tax*magneticAmplitude*Math.cos(theta+Math.PI/12-advance));
        }
    }
    traveling.setState(travelingFields);
    for (let i=0;i<steps;++i) traveling.step(th);
    sameFields(traveling.getState(),expectedFields,8e-13); traveling.delete();
    const c={...base, columns:12, rows:10, spacingX:.23, spacingY:.41, maxSubstep:10};
    const grid=new physics.MaxwellGrid(c), h=.8*grid.getStableTimeStep();
    const initial=mode(c,3,2,.7,-.13);
    // Add DC and a curl-null longitudinal magnetic component with nonzero div H.
    for (let j=0; j<c.rows; ++j) for (let i=0; i<c.columns; ++i) {
        const k=i+c.columns*j;
        initial.ez[k]+=.4; initial.hx[k]+=1.2+.2*Math.sin(2*Math.PI*(i+.5)/c.columns);
        initial.hy[k]-=.3;
    }
    grid.setState(initial);
    initial.ez[0]=99; assert.notEqual(grid.getState().ez[0],99);
    const divergence=grid.getMagneticDivergence(), invariant=grid.getModifiedEnergy(h);
    assert.ok(Math.max(...divergence.map(Math.abs))>.1);
    const energy0=grid.getDiagnostics().totalEnergy;
    let minEnergy=energy0,maxEnergy=energy0;
    for (let n=0; n<300; ++n) {
        grid.step(h); const diag=grid.getDiagnostics();
        near(diag.modifiedEnergy,invariant,2e-11*invariant);
        minEnergy=Math.min(minEnergy,diag.totalEnergy); maxEnergy=Math.max(maxEnergy,diag.totalEnergy);
    }
    assert.ok(maxEnergy-minEnergy>1e-5);
    const diag=grid.getDiagnostics();
    assert.deepEqual(Object.keys(diag).sort(), ['electricEnergy','magneticEnergy','totalEnergy',
        'modifiedEnergy','modifiedEnergyStep','meanEz','meanHx','meanHy','maxAbsEz','maxAbsHx',
        'maxAbsHy','magneticDivergenceRms','maxAbsMagneticDivergence','time','stableTimeStep',
        'lastSubstep','lastSubsteps','lastCellVisits'].sort());
    near(diag.meanEz,.4); near(diag.meanHx,1.2); near(diag.meanHy,-.3);
    const finalDivergence=grid.getMagneticDivergence();
    for (let i=0; i<divergence.length; ++i) near(finalDivergence[i],divergence[i],2e-13);
    const rms=Math.hypot(...finalDivergence)/Math.sqrt(finalDivergence.length);
    near(diag.magneticDivergenceRms,rms,1e-15);
    near(diag.maxAbsMagneticDivergence,Math.max(...finalDivergence.map(Math.abs)),1e-15);
    near(diag.totalEnergy,diag.electricEnergy+diag.magneticEnergy,1e-14);
    const before=grid.getState(), beforeDiag=grid.getDiagnostics();
    const unchanged=()=>{assert.deepEqual(grid.getState(),before); assert.deepEqual(grid.getDiagnostics(),beforeDiag);};
    grid.step(0); unchanged();
    for (const bad of [-1,NaN,Infinity,Number.MAX_VALUE,Number.MIN_VALUE]) {
        assert.throws(()=>grid.step(bad)); unchanged();
    }
    const physicalCfl=1/(grid.getWaveSpeed()*Math.hypot(1/c.spacingX,1/c.spacingY));
    for (const bad of [-1,NaN,Infinity,physicalCfl*1.000001,Number.MIN_VALUE]) {
        assert.throws(()=>grid.getModifiedEnergy(bad)); unchanged();
    }
    near(grid.getModifiedEnergy(0),beforeDiag.totalEnergy,0);
    for (const field of ['ez','hx','hy']) {
        for (const bad of [[],Array(before.ez.length-1).fill(0),Array(before.ez.length+1).fill(0),
            Array(2**32-1),new Float64Array(before.ez.length),null]) {
            assert.throws(()=>grid.setState({...before,[field]:bad})); unchanged();
        }
        for (const bad of [NaN,Infinity,-Infinity,'1',undefined,null,{},1e308]) {
            const values=before[field].slice(); values[1]=bad;
            assert.throws(()=>grid.setState({...before,[field]:values})); unchanged();
        }
        const hole=before[field].slice(); delete hole[1];
        assert.throws(()=>grid.setState({...before,[field]:hole})); unchanged();
        // An inherited numeric entry must not make a sparse array dense.
        const prototype=Object.create(Array.prototype); prototype[1]=0;
        Object.setPrototypeOf(hole,prototype);
        assert.throws(()=>grid.setState({...before,[field]:hole})); unchanged();
    }
    const readTrap=before.ez.slice(); Object.defineProperty(readTrap,0,{get(){throw Error('entries read before lengths');}});
    assert.throws(()=>grid.setState({ez:readTrap,hx:before.hx,hy:[]}),error=>error.message!=='entries read before lengths');
    unchanged();
    for (const bad of [{},null,{ez:before.ez,hx:before.hx}]) {
        assert.throws(()=>grid.setState(bad)); unchanged();
    }
    const stateSnapshot=grid.getState(), configSnapshot=grid.getConfig(), diagSnapshot=grid.getDiagnostics();
    const divSnapshot=grid.getMagneticDivergence();
    stateSnapshot.ez[0]=99; configSnapshot.columns=99; diagSnapshot.time=99; divSnapshot[0]=99;
    unchanged(); assert.equal(grid.getConfig().columns,12); assert.notEqual(grid.getMagneticDivergence()[0],99);
    grid.setState(before); assert.equal(grid.getDiagnostics().time,beforeDiag.time);
    assert.equal(grid.getDiagnostics().lastCellVisits,0); assert.equal(grid.getDiagnostics().modifiedEnergyStep,0);
    const retained=grid.getState(); grid.delete();
    assert.deepEqual(retained,before); assert.equal(stateSnapshot.ez[0],99); assert.equal(diagSnapshot.time,99);
    assert.equal(divSnapshot[0],99); assert.equal(configSnapshot.columns,99);
    // Uniform mode has no curl: physical SI energy and DC fields remain unchanged.
    const dc=new physics.MaxwellGrid({...base,columns:2,rows:3,spacingX:.25,spacingY:.5,permittivity:2,permeability:3});
    const uniform={ez:Array(6).fill(2),hx:Array(6).fill(3),hy:Array(6).fill(-4)};
    dc.setState(uniform); dc.step(.3); assert.deepEqual(dc.getState(),uniform);
    near(dc.getDiagnostics().electricEnergy,3); near(dc.getDiagnostics().magneticEnergy,28.125);
    near(dc.getDiagnostics().totalEnergy,31.125); assert.deepEqual(dc.getMagneticDivergence(),Array(6).fill(0)); dc.delete();
    for (const field of ['columns','rows','maximumSubsteps','maximumCellVisits'])
        for (const bad of [-1,0,.5,NaN,Infinity,2**32,2**32+1,Number.MAX_SAFE_INTEGER])
            assert.throws(()=>new physics.MaxwellGrid({...base,[field]:bad}));
    for (const [field,bad] of [['columns',262145],['rows',1],['maximumSubsteps',1000001],['maximumCellVisits',1000000001]])
        assert.throws(()=>new physics.MaxwellGrid({...base,[field]:bad}));
    assert.throws(()=>new physics.MaxwellGrid({...base,columns:512,rows:513}));
    for (const field of ['spacingX','spacingY','permittivity','permeability','maxSubstep'])
        for (const bad of [-1,0,NaN,Infinity])
            assert.throws(()=>new physics.MaxwellGrid({...base,[field]:bad}));
    for (const field of ['spacingX','spacingY','permittivity','permeability'])
        assert.throws(()=>new physics.MaxwellGrid({...base,[field]:Number.MIN_VALUE}));
    const tinyDuration=new physics.MaxwellGrid({...base,maxSubstep:Number.MIN_VALUE});
    const tinyDiag=tinyDuration.getDiagnostics(),tinyState=tinyDuration.getState();
    assert.throws(()=>tinyDuration.step(Number.MIN_VALUE));
    assert.deepEqual(tinyDuration.getDiagnostics(),tinyDiag); assert.deepEqual(tinyDuration.getState(),tinyState);
    tinyDuration.delete();
    const maximalBudgets=new physics.MaxwellGrid({...base,columns:2,rows:2,
        maximumSubsteps:1000000,maximumCellVisits:1000000000});
    assert.equal(maximalBudgets.getConfig().maximumSubsteps,1000000);
    assert.equal(maximalBudgets.getConfig().maximumCellVisits,1000000000); maximalBudgets.delete();
    for (const bad of [0,-1,1,NaN,Infinity]) assert.throws(()=>new physics.MaxwellGrid({...base,cflSafety:bad}));
    // An exact decimal user limit must consume one substep and exactly four cell passes.
    const bounded=new physics.MaxwellGrid({...base,columns:2,rows:2,maxSubstep:.01,maximumSubsteps:1,maximumCellVisits:16});
    bounded.step(.01); assert.equal(bounded.getDiagnostics().lastSubsteps,1);
    assert.equal(bounded.getDiagnostics().lastCellVisits,16);
    const bd=bounded.getDiagnostics(),bs=bounded.getState();
    assert.throws(()=>bounded.step(.011)); assert.deepEqual(bounded.getState(),bs); assert.deepEqual(bounded.getDiagnostics(),bd);
    bounded.delete();
    const noWork=new physics.MaxwellGrid({...base,columns:2,rows:2,maximumCellVisits:15});
    const noDiag=noWork.getDiagnostics(),noState=noWork.getState(); assert.throws(()=>noWork.step(.01));
    assert.deepEqual(noWork.getDiagnostics(),noDiag); assert.deepEqual(noWork.getState(),noState); noWork.delete();
    const overflow=new physics.MaxwellGrid({...base,columns:2,rows:2,maxSubstep:1});
    const large=mode(overflow.getConfig(),1,1,0,3e153); overflow.setState(large);
    const largeDiag=overflow.getDiagnostics(); assert.throws(()=>overflow.step(.6));
    assert.deepEqual(overflow.getState(),large); assert.deepEqual(overflow.getDiagnostics(),largeDiag); overflow.delete();
    defaults.delete();
}

function testWaves(physics) {
    const defaults = new physics.WaveMembrane(3, 3, 1, 1);
    const config = defaults.getConfig();
    assert.equal(config.boundary, physics.WaveBoundary.FixedZero);
    assert.equal(config.maxCells, 1000000);
    config.boundary = physics.WaveBoundary.Periodic;
    config.maxSubstep = 0.001; config.tension = 6; config.surfaceDensity = 2;
    assert.equal(defaults.getConfig().tension, 1);
    const mode = new physics.WaveMembrane(8, 6, 0.3, 0.4, config);
    assert.equal(mode.getWidth(), 8); assert.equal(mode.getHeight(), 6); assert.equal(mode.getCellCount(), 48);
    assert.equal(mode.getSpacingX(), 0.3); assert.equal(mode.getSpacingY(), 0.4);
    const initial = Array.from({length: 48}, (_, i) => Math.cos(2 * Math.PI * ((i % 8) / 8 + Math.floor(i / 8) / 6)));
    const velocities = Array(48).fill(0);
    mode.setState(initial, velocities);
    const omega = 2 * Math.sqrt(3) * Math.hypot(Math.sin(Math.PI / 8) / 0.3, Math.sin(Math.PI / 6) / 0.4);
    const initialEnergy = mode.getDiagnostics().totalEnergy;
    assert.ok(Math.abs(initialEnergy - 0.25 * 2 * 0.3 * 0.4 * 48 * omega**2) < 1e-12);
    const inputCopy = initial.slice(); mode.setState(inputCopy, velocities); inputCopy[0] = 99;
    assert.equal(mode.getCell(0, 0).displacement, initial[0]);
    mode.step(0.2);
    const diag = mode.getDiagnostics(), state = mode.getDisplacements(), velocity = mode.getVelocities();
    assert.equal(diag.lastCellWork, 48 * diag.lastSubsteps);
    assert.equal(diag.lastSubsteps, 200); assert.equal(diag.lastSubstep, 0.001);
    assert.equal(diag.time, 0.2); assert.ok(diag.lastSubstep <= mode.getStableTimeStep());
    const phase = 2 * Math.asin(omega * diag.lastSubstep / 2) * diag.lastSubsteps;
    const amplitude = Math.cos(phase), speedAmplitude = -omega * Math.sqrt(1 - (omega * diag.lastSubstep / 2)**2) * Math.sin(phase);
    for (let i = 0; i < 48; ++i) {
        assert.ok(Math.abs(state[i] - initial[i] * amplitude) < 2e-13);
        assert.ok(Math.abs(velocity[i] - initial[i] * speedAmplitude) < 1e-12);
    }
    assert.ok(diag.totalEnergy <= initialEnergy * (1 + 1e-12));
    assert.ok(diag.totalEnergy > initialEnergy * 0.99);
    const cellSnapshot = mode.getCell(1, 2), arraySnapshot = mode.getDisplacements(), loadSnapshot = mode.getQueuedAccelerations();
    cellSnapshot.displacement = 99; arraySnapshot[0] = 99; loadSnapshot[0] = 99; diag.totalEnergy = 99;
    assert.notEqual(mode.getCell(1, 2).displacement, 99); assert.notEqual(mode.getDisplacements()[0], 99);
    assert.equal(mode.getCell(0, 0).queuedAcceleration, 0); assert.notEqual(mode.getDiagnostics().totalEnergy, 99);
    for (const bad of [-1, 0.5, NaN, Infinity, 2**32, 2**32 + 1]) {
        assert.throws(() => new physics.WaveMembrane(bad, 3, 1, 1));
        assert.throws(() => new physics.WaveMembrane(3, bad, 1, 1, config));
        for (const [x, y] of [[bad, 0], [0, bad]]) {
            assert.throws(() => mode.getCell(x, y));
            assert.throws(() => mode.setCellState(x, y, 0, 0));
            assert.throws(() => mode.queueAcceleration(x, y, 1));
            assert.throws(() => mode.clearAcceleration(x, y));
        }
    }
    assert.throws(() => mode.getCell(8, 0)); assert.throws(() => mode.getCell(0, 6));
    for (const field of ["maxCells", "maxSubsteps", "maxCellWork"]) {
        for (const count of [-1, 0, 0.5, NaN, Infinity, 2**32, 2**32 + 1]) {
            const invalid = {...config, [field]: count};
            assert.throws(() => mode.setConfig(invalid));
            assert.throws(() => new physics.WaveMembrane(8, 6, 0.3, 0.4, invalid));
            assert.equal(mode.getConfig()[field], config[field]);
        }
    }
    assert.throws(() => mode.setConfig({...config, maxCells: 47}));
    assert.throws(() => mode.setConfig({...config, boundary: 99}));
    assert.throws(() => new physics.WaveMembrane(0, 3, 1, 1));
    assert.throws(() => new physics.WaveMembrane(2, 2, 1, 1));
    assert.throws(() => new physics.WaveMembrane(2**31, 2, 1, 1, {...config, maxCells: 2**32 - 1}));
    for (const spacing of [0, -1, NaN, Infinity]) assert.throws(() => new physics.WaveMembrane(3, 3, spacing, 1));
    const beforeInvalid = mode.getDisplacements();
    for (const values of [null, {}, {length: 48}, new Float64Array(48), Array(47).fill(0), new Array(2**32 - 1)])
        assert.throws(() => mode.setState(values, velocities));
    assert.throws(() => mode.setState(beforeInvalid, []));
    assert.throws(() => mode.setState(Array(48), velocities)); // Sparse entries are not numbers.
    assert.throws(() => mode.setState(["1", ...beforeInvalid.slice(1)], velocities));
    assert.throws(() => mode.setState([NaN, ...beforeInvalid.slice(1)], velocities));
    assert.throws(() => mode.setState(Array(48).fill(3), [Infinity, ...velocities.slice(1)]));
    assert.deepEqual(mode.getDisplacements(), beforeInvalid);
    assert.throws(() => mode.setCellState(0, 0, NaN)); assert.throws(() => mode.queueAcceleration(0, 0, Infinity));

    const uniformConfig = {...config, tension: 1, surfaceDensity: 2, maxSubstep: 0.01};
    const uniform = new physics.WaveMembrane(2, 3, 1, 1, uniformConfig);
    uniform.setState(Array(6).fill(3), Array(6).fill(-2));
    for (let y = 0; y < 3; ++y) for (let x = 0; x < 2; ++x) uniform.queueAcceleration(x, y, 4);
    uniform.step(0); assert.deepEqual(uniform.getQueuedAccelerations(), Array(6).fill(4));
    const beforeZero = uniform.getCell(0, 0), beforeDiag = uniform.getDiagnostics();
    for (const dt of [-1, NaN, Infinity, 100]) {
        assert.throws(() => uniform.step(dt)); assert.deepEqual(uniform.getCell(0, 0), beforeZero);
        assert.deepEqual(uniform.getDiagnostics(), beforeDiag);
    }
    uniform.setConfig({...uniformConfig, maxCellWork: 6});
    assert.throws(() => uniform.step(0.5)); assert.deepEqual(uniform.getQueuedAccelerations(), Array(6).fill(4));
    uniform.setConfig({...uniformConfig, maxSubsteps: 1});
    assert.throws(() => uniform.step(0.5)); assert.deepEqual(uniform.getCell(0, 0), beforeZero);
    uniform.setConfig(uniformConfig); uniform.step(0.5);
    for (const cell of uniform.getDisplacements()) assert.ok(Math.abs(cell - 2.5) < 1e-13);
    for (const speed of uniform.getVelocities()) assert.ok(Math.abs(speed) < 1e-13);
    assert.deepEqual(uniform.getQueuedAccelerations(), Array(6).fill(0));
    uniform.queueAcceleration(0, 0, 1e308);
    const overflowState = uniform.getDisplacements(), overflowDiag = uniform.getDiagnostics();
    assert.throws(() => uniform.step(0.01)); assert.deepEqual(uniform.getDisplacements(), overflowState);
    assert.deepEqual(uniform.getDiagnostics(), overflowDiag); assert.equal(uniform.getCell(0, 0).queuedAcceleration, 1e308);
    uniform.clearAcceleration(0, 0); uniform.queueAcceleration(0, 0, 1); uniform.clearAccelerations();
    assert.deepEqual(uniform.getQueuedAccelerations(), Array(6).fill(0));
    uniform.setConfig({...uniformConfig, damping: 0.7}); uniform.setState(Array(6).fill(0), Array(6).fill(2));
    uniform.step(0.5);
    for (const speed of uniform.getVelocities()) assert.ok(Math.abs(speed - 2 * Math.exp(-0.7)) < 2e-14);
    assert.ok(uniform.getDiagnostics().kineticEnergy < 6);

    defaults.setCellState(1, 1, 0.01); defaults.queueAcceleration(1, 1, 2); defaults.step(0.01);
    for (const [x, y] of [[0, 0], [0, 1], [2, 1], [1, 0], [1, 2]]) {
        assert.equal(defaults.getCell(x, y).displacement, 0); assert.equal(defaults.getCell(x, y).velocity, 0);
        assert.throws(() => defaults.setCellState(x, y, 1));
        assert.throws(() => defaults.setCellState(x, y, 0, 1));
        assert.throws(() => defaults.queueAcceleration(x, y, 1));
    }
    const edgesBefore = defaults.getDisplacements();
    assert.throws(() => defaults.setState(Array(9).fill(1), Array(9).fill(0)));
    assert.deepEqual(defaults.getDisplacements(), edgesBefore);
    defaults.setConfig({...defaults.getConfig(), boundary: physics.WaveBoundary.Periodic}); defaults.queueAcceleration(0, 0, 1);
    assert.throws(() => defaults.setConfig({...defaults.getConfig(), boundary: physics.WaveBoundary.FixedZero}));
    assert.equal(defaults.getConfig().boundary, physics.WaveBoundary.Periodic);
    defaults.clearAccelerations(); defaults.setConfig({...defaults.getConfig(), boundary: physics.WaveBoundary.FixedZero});
    mode.delete(); uniform.delete(); defaults.delete();
    assert.equal(cellSnapshot.displacement, 99); assert.equal(arraySnapshot[0], 99); assert.equal(loadSnapshot[0], 99);
    assert.equal(diag.totalEnergy, 99); assert.equal(config.maxSubstep, 0.001);
}

function testGravity(physics) {
    const defaults = new physics.NBodyGravity();
    const config = defaults.getConfig();
    assert.equal(config.gravitationalStrength, 1);
    config.maxSubstep = 0.002;
    assert.equal(defaults.getConfig().maxSubstep, 0.01);
    defaults.delete();
    const orbit = new physics.NBodyGravity(config);
    assert.equal(orbit.addParticle({x: -0.5, y: 0}, {x: 0, y: -Math.SQRT1_2}, 1), 0);
    assert.equal(orbit.addParticle({x: 0.5, y: 0}, {x: 0, y: Math.SQRT1_2}, 1), 1);
    assert.equal(orbit.getParticleCount(), 2);
    orbit.step(0.5);
    const p = orbit.getParticle(0), d = orbit.getDiagnostics();
    assert.ok(Math.hypot(p.position.x + 0.5 * Math.cos(Math.SQRT2 * 0.5),
                         p.position.y + 0.5 * Math.sin(Math.SQRT2 * 0.5)) < 3e-6);
    assert.ok(Math.abs(d.totalEnergy + 0.5) < 2e-6);
    assert.ok(Math.hypot(d.momentum.x, d.momentum.y) < 1e-13);
    assert.ok(d.lastSubsteps >= 250 && d.lastPairWork >= d.lastSubsteps);
    p.position.x = 99; d.centerOfMass.y = 99;
    assert.notEqual(orbit.getParticle(0).position.x, 99);
    assert.notEqual(orbit.getDiagnostics().centerOfMass.y, 99);
    const before = orbit.getParticle(0);
    for (const index of [-1, 0.5, NaN, Infinity, 2, 2**32]) {
        assert.throws(() => orbit.getParticle(index));
        assert.throws(() => orbit.setState(index, {x: 0, y: 0}, {x: 0, y: 0}));
        assert.throws(() => orbit.applyImpulse(index, {x: 0, y: 0}));
    }
    assert.throws(() => orbit.setState(0, {x: Infinity, y: 0}, {x: 0, y: 0}));
    for (const key of ["maxParticles", "maxSubsteps", "maxPairWork"])
        for (const value of [-1, 0, 1.5, NaN, Infinity, 2**32])
            assert.throws(() => orbit.setConfig({...config, [key]: value}));
    assert.throws(() => new physics.NBodyGravity({...config, maxPairWork: 0.5}));
    assert.deepEqual(orbit.getParticle(0), before);
    assert.deepEqual(orbit.getConfig(), config);
    orbit.setConfig({...config, maxPairWork: 1});
    assert.throws(() => orbit.step(0.002));
    assert.deepEqual(orbit.getParticle(0), before);
    assert.equal(orbit.getDiagnostics().lastPairWork, d.lastPairWork);
    orbit.setConfig(config);
    orbit.step(0);
    assert.deepEqual(orbit.getParticle(0), before);
    assert.equal(orbit.getDiagnostics().lastPairWork, 0);
    orbit.delete();
    assert.equal(p.position.x, 99); // Plain snapshot remains usable after deletion.
    assert.equal(before.mass, 1);

    const free = new physics.NBodyGravity({...config, gravitationalStrength: 0});
    free.addParticle({x: 1e10, y: 0});
    free.addParticle({x: 1e10, y: 0}); // G=0 coincidence is allowed.
    free.setState(0, {x: 1e10, y: 0}, {x: 0, y: 2});
    free.applyImpulse(0, {x: 0, y: 1});
    free.step(0.1);
    assert.ok(Math.abs(free.getParticle(0).position.y - 0.3) < 1e-13);
    assert.equal(free.getParticle(0).velocity.y, 3);
    free.delete();

    const singular = new physics.NBodyGravity(config);
    singular.addParticle({x: 0, y: 0}); singular.addParticle({x: 0, y: 0});
    assert.throws(() => singular.step(0.002));
    assert.equal(singular.getParticle(0).position.x, 0);
    singular.setConfig({...config, softening: 0.1});
    singular.step(0.002);
    assert.equal(singular.getParticle(0).velocity.x, 0);
    assert.ok(Math.abs(singular.getDiagnostics().potentialEnergy + 10) < 1e-12);
    singular.delete();
}

function testMacProjection(physics) {
    const near = (actual, expected, tolerance = 2e-12) => assert.ok(Math.abs(actual - expected) <= tolerance,
        `MAC: ${actual} != ${expected} within ${tolerance}`);
    const mean = a => a.reduce((sum, x) => sum + x / a.length, 0);
    const rms = a => Math.sqrt(a.reduce((sum, x) => sum + x*x / a.length, 0));
    const options = {density: 3, timeStep: 0.2, absoluteDivergenceTolerance: 1e-10,
        relativeDivergenceTolerance: 1e-10, maximumIterations: 1000, maximumCellVisits: 100000000};
    const mode = (c, mx, my, u, v, meanX = 0, meanY = 0) => {
        const xFaces = [], yFaces = [];
        for (let j = 0; j < c.rows; ++j) for (let i = 0; i < c.columns; ++i) {
            xFaces.push(meanX + u*Math.sin(2*Math.PI*(mx*i/c.columns + my*(j+0.5)/c.rows) + 0.31));
            yFaces.push(meanY + v*Math.sin(2*Math.PI*(mx*(i+0.5)/c.columns + my*j/c.rows) + 0.31));
        }
        return {xFaces, yFaces};
    };
    // Independently derived staggered Fourier symbol, including duplicate
    // forward/backward edges on either two-cell periodic axis.
    for (const [columns, rows] of [[12,10], [2,5], [5,2], [2,2]]) {
        const c = {columns, rows, spacingX: 0.23, spacingY: 0.41};
        const grid = new physics.PeriodicMacGrid(c);
        const ax = 2*Math.sin(Math.PI/columns)/c.spacingX, ay = 2*Math.sin(Math.PI/rows)/c.spacingY;
        const eigenvalue = ax*ax + ay*ay;
        const longitudinal = mode(c, 1, 1, ax, ay, 0.17, -0.28);
        grid.setVelocities(longitudinal.xFaces, longitudinal.yFaces);
        const divergence = grid.getDivergence();
        for (let j = 0; j < rows; ++j) for (let i = 0; i < columns; ++i) {
            const phase = 2*Math.PI*((i+0.5)/columns + (j+0.5)/rows) + 0.31;
            near(divergence[i+columns*j], eigenvalue*Math.cos(phase));
        }
        const d = grid.project(options), p = grid.getLastProjection(), v = grid.getVelocities();
        assert.ok(d.iterations <= 2); assert.ok(d.cellVisits > 0);
        assert.deepEqual(d, p.diagnostics); assert.equal(d.density, 3); assert.equal(d.timeStep, 0.2);
        assert.ok(d.finalDivergenceRms <= d.targetDivergenceRms);
        near(d.finalMeanX, 0.17); near(d.finalMeanY, -0.28);
        near(d.potentialMean, 0); near(d.pressureMean, 0);
        for (let j = 0; j < rows; ++j) for (let i = 0; i < columns; ++i) {
            const k = i+columns*j, phase = 2*Math.PI*((i+0.5)/columns + (j+0.5)/rows) + 0.31;
            near(p.potential[k], -Math.cos(phase)); near(p.pressure[k], -15*Math.cos(phase), 3e-11);
            near(v.xFaces[k], 0.17); near(v.yFaces[k], -0.28);
        }
        const transverse = mode(c, 1, 1, ay, -ax, 0.71, -0.27);
        grid.setVelocities(transverse.xFaces, transverse.yFaces);
        const td = grid.project(), tv = grid.getVelocities();
        for (let k = 0; k < columns*rows; ++k) {
            near(tv.xFaces[k], transverse.xFaces[k]); near(tv.yFaces[k], transverse.yFaces[k]);
        }
        near(td.finalMeanX, 0.71); near(td.finalMeanY, -0.27); near(td.finalDivergenceRms, 0);
        grid.delete();
    }
    const c = {columns: 17, rows: 12, spacingX: 0.17, spacingY: 0.31};
    const grid = new physics.PeriodicMacGrid(c), n = c.columns*c.rows;
    const input = mode(c, 1, 2, 0.7, -0.3, 0.17, -0.28), second = mode(c, 3, 1, -0.2, 0.8);
    for (let k = 0; k < n; ++k) { input.xFaces[k] += second.xFaces[k]; input.yFaces[k] += second.yFaces[k]; }
    grid.setVelocities(input.xFaces, input.yFaces);
    const loose = {...options, density: 7, timeStep: 0.031, relativeDivergenceTolerance: 0,
        absoluteDivergenceTolerance: rms(grid.getDivergence())*0.65};
    const d = grid.project(loose), p = grid.getLastProjection(), div = grid.getDivergence();
    near(d.initialKineticEnergy, 0.5*loose.density*c.spacingX*c.spacingY*
        [...input.xFaces, ...input.yFaces].reduce((sum, x) => sum+x*x, 0), 2e-13);
    near(d.finalDivergenceRms, rms(div)); assert.ok(d.finalDivergenceRms > 1e-8);
    assert.ok(d.finalDivergenceRms <= loose.absoluteDivergenceTolerance);
    near(d.divergencePotentialInnerProduct, loose.density*c.spacingX*c.spacingY*
        div.reduce((sum, x, i) => sum+x*p.potential[i], 0), 2e-13);
    assert.ok(Math.abs(d.velocityCorrectionInnerProduct) <= d.residualEnergyBound+d.roundoffEnergyAllowance);
    assert.ok(Math.abs(d.velocityCorrectionInnerProduct+d.divergencePotentialInnerProduct) <= d.roundoffEnergyAllowance);
    assert.ok(Math.abs(d.storageEnergyError) <= d.roundoffEnergyAllowance);
    assert.ok(d.finalKineticEnergy <= d.initialKineticEnergy+d.residualEnergyBound+d.roundoffEnergyAllowance);
    near(d.finalMeanX, d.initialMeanX); near(d.finalMeanY, d.initialMeanY);
    near(mean(p.potential), 0); near(mean(p.pressure), 0);
    for (let k = 0; k < n; ++k) near(p.pressure[k], loose.density/loose.timeStep*p.potential[k]);

    // Budget exactly sufficient must succeed; one cell visit less fails late
    // in the staged solve and retains both arrays and all prior diagnostics.
    grid.setVelocities(input.xFaces, input.yFaces);
    const sufficient = grid.project(options);
    grid.setVelocities(input.xFaces, input.yFaces);
    assert.deepEqual(grid.project({...options, maximumCellVisits: sufficient.cellVisits}), sufficient);
    grid.setVelocities(input.xFaces, input.yFaces);
    const beforeV = grid.getVelocities(), beforeP = grid.getLastProjection();
    const unchanged = () => { assert.deepEqual(grid.getVelocities(), beforeV); assert.deepEqual(grid.getLastProjection(), beforeP); };
    for (const fail of [ {...options, maximumIterations: 0}, {...options, maximumIterations: 1},
        {...options, maximumCellVisits: 0}, {...options, maximumCellVisits: sufficient.cellVisits-1},
        {...options, timeStep: 0}, {...options, density: 1e300, timeStep: 1e-300},
        {...options, absoluteDivergenceTolerance: -1}, {...options, relativeDivergenceTolerance: NaN} ]) {
        assert.throws(() => grid.project(fail)); unchanged();
    }
    const constantX = Array(n).fill(0.7), constantY = Array(n).fill(-0.2);
    grid.setVelocities(constantX, constantY);
    const zero = grid.project({...options, maximumIterations: 0});
    assert.equal(zero.zeroDivergenceNoOp, true); assert.equal(zero.iterations, 0);
    assert.deepEqual(grid.getDivergence(), Array(n).fill(0));
    assert.deepEqual(grid.getVelocities(), {xFaces: constantX, yFaces: constantY});
    assert.deepEqual(grid.getLastProjection().potential, Array(n).fill(0));
    assert.deepEqual(grid.getLastProjection().pressure, Array(n).fill(0));
    const countBeforeV = grid.getVelocities(), countBeforeP = grid.getLastProjection();
    const countUnchanged = () => { assert.deepEqual(grid.getVelocities(), countBeforeV); assert.deepEqual(grid.getLastProjection(), countBeforeP); };
    for (const field of ["maximumIterations", "maximumCellVisits"]) {
        for (const bad of [-1, 0.5, NaN, Infinity, -Infinity, 2**32, 2**32+100000, Number.MAX_VALUE]) {
            assert.throws(() => grid.project({...options, [field]: bad})); countUnchanged();
        }
        assert.throws(() => grid.project({...options, [field]: field === "maximumIterations" ? 1000001 : 1000000001}));
        countUnchanged();
    }
    assert.equal(grid.project({...options, maximumIterations: 1000000, maximumCellVisits: 1000000000}).zeroDivergenceNoOp, true);
    const constantBefore = grid.getVelocities(), projectionBefore = grid.getLastProjection();
    const setterUnchanged = () => { assert.deepEqual(grid.getVelocities(), constantBefore); assert.deepEqual(grid.getLastProjection(), projectionBefore); };
    for (const bad of [[], Array(n+1).fill(0), new Float64Array(n), {length:n}, null, Array(2**32-1)]) {
        assert.throws(() => grid.setVelocities(bad, constantY)); setterUnchanged();
        assert.throws(() => grid.setVelocities(constantX, bad)); setterUnchanged();
    }
    for (const bad of ["1", undefined, null, {}, NaN, Infinity, -Infinity, 1n]) {
        const x = constantX.slice(), y = constantY.slice(); x[n-1] = bad;
        assert.throws(() => grid.setVelocities(x, y)); setterUnchanged();
        x[n-1] = 0.7; y[n-1] = bad;
        assert.throws(() => grid.setVelocities(x, y)); setterUnchanged();
    }
    let reads = 0;
    const watched = constantX.slice(); Object.defineProperty(watched, 0, {get() { ++reads; throw Error("must not read"); }});
    assert.throws(() => grid.setVelocities(watched, [])); assert.equal(reads, 0); setterUnchanged();
    const sparse = Array(n); assert.throws(() => grid.setVelocities(sparse, constantY)); setterUnchanged();

    const copyConfig = grid.getConfig(), copyV = grid.getVelocities(), copyP = grid.getLastProjection(), copyDiv = grid.getDivergence();
    assert.ok(Array.isArray(copyV.xFaces)); assert.ok(Array.isArray(copyP.pressure)); assert.ok(Array.isArray(copyDiv));
    assert.equal(Object.getPrototypeOf(copyP), Object.prototype); assert.equal(typeof copyP.diagnostics.delete, "undefined");
    assert.equal(Object.getPrototypeOf(copyP.diagnostics), Object.prototype);
    copyConfig.columns = 99; copyV.xFaces[0] = 99; copyP.potential[0] = 99; copyP.diagnostics.cellVisits = 99; copyDiv[0] = 99;
    assert.deepEqual(grid.getConfig(), c); setterUnchanged();
    constantX[0] = 99; constantY[0] = 99; setterUnchanged();
    const huge = Array(n).fill(Number.MAX_VALUE); grid.setVelocities(huge, huge);
    const hugeBefore = grid.getVelocities(); assert.throws(() => grid.project());
    assert.deepEqual(grid.getVelocities(), hugeBefore); assert.deepEqual(grid.getLastProjection(), projectionBefore);
    grid.delete();
    assert.equal(copyV.xFaces[1], 0.7); assert.equal(copyP.pressure[1], 0); assert.equal(copyP.diagnostics.zeroDivergenceNoOp, true);
    assert.equal(copyConfig.rows, c.rows); assert.equal(copyDiv[1], 0);

    const defaults = new physics.PeriodicMacGrid();
    const defaultConfig = defaults.getConfig(); assert.deepEqual(defaultConfig, {columns:16, rows:16, spacingX:1, spacingY:1});
    assert.equal(defaults.project().zeroDivergenceNoOp, true); defaults.delete();
    for (const field of ["columns", "rows"]) for (const bad of [0, 1, -1, 2.5, NaN, Infinity, 2**32, 2**32+2, 262145, Number.MAX_VALUE])
        assert.throws(() => new physics.PeriodicMacGrid({...defaultConfig, [field]:bad}));
    assert.throws(() => new physics.PeriodicMacGrid({...defaultConfig, columns:65536, rows:65536}));
    for (const field of ["spacingX", "spacingY"]) for (const bad of [0, -1, NaN, Infinity, 1e-200, 1e200])
        assert.throws(() => new physics.PeriodicMacGrid({...defaultConfig, [field]:bad}));
    const largest = new physics.PeriodicMacGrid({columns:512, rows:512, spacingX:1, spacingY:1});
    assert.equal(largest.getConfig().columns*largest.getConfig().rows, 262144); largest.delete();
}

function testMacDiffusion(physics) {
    const near = (actual, expected, tolerance=3e-14) => assert.ok(Math.abs(actual-expected)<=tolerance,
        `MAC diffusion: ${actual} != ${expected} within ${tolerance}`);
    const options = {kinematicViscosity:0.17, timeStep:0.3, density:3,
        absoluteVelocityTolerance:1e-12, relativeVelocityTolerance:0, maximumIterations:1000, maximumCellVisits:100000000};
    const fields = ["iterations", "iterationsX", "iterationsY", "cellVisits", "kinematicViscosity", "timeStep", "density",
        "initialVelocityRms", "finalResidualRms", "targetResidualRms", "initialMeanX", "initialMeanY", "finalMeanX", "finalMeanY",
        "meanRoundoffAllowanceX", "meanRoundoffAllowanceY", "initialKineticEnergy", "finalKineticEnergy", "gradientDissipation",
        "incrementKineticEnergy", "residualWork", "residualEnergyBound", "storageEnergyError", "roundoffEnergyAllowance", "zeroTransportNoOp"];
    const mode = (c,mx,my,u,v) => {
        const xFaces=[], yFaces=[];
        for(let j=0;j<c.rows;++j) for(let i=0;i<c.columns;++i) {
            xFaces.push(u*Math.sin(2*Math.PI*(mx*i/c.columns+my*(j+0.5)/c.rows)+0.31));
            yFaces.push(v*Math.sin(2*Math.PI*(mx*(i+0.5)/c.columns+my*j/c.rows)+0.31));
        }
        return {xFaces,yFaces};
    };
    const lambda = (c,mx,my) => 4*(Math.sin(Math.PI*mx/c.columns)/c.spacingX)**2+4*(Math.sin(Math.PI*my/c.rows)/c.spacingY)**2;
    const auditDiagnostics = d => {
        assert.deepEqual(Object.keys(d).sort(), fields.slice().sort());
        assert.equal(Object.getPrototypeOf(d),Object.prototype);
        assert.equal(typeof d.delete,"undefined");
        for(const field of fields) {
            if(field==="zeroTransportNoOp") assert.equal(typeof d[field],"boolean");
            else assert.ok(Number.isFinite(d[field]),`${field} must be finite`);
        }
        assert.equal(d.iterations,d.iterationsX+d.iterationsY);
        assert.ok(d.finalResidualRms<=d.targetResidualRms);
    };
    // Independent discrete Fourier eigenvalue: each two-cell axis contributes
    // 4/spacing^2 for the alternating mode, rather than counting its neighbor once.
    for(const [columns,rows] of [[13,11],[2,5],[5,2],[2,2]]) {
        const c={columns,rows,spacingX:0.23,spacingY:0.41}, grid=new physics.PeriodicMacGrid(c);
        const initial=mode(c,1,1,0.7,-0.4); grid.setVelocities(initial.xFaces,initial.yFaces);
        const d=grid.diffuse(options), actual=grid.getVelocities(); auditDiagnostics(d);
        assert.deepEqual(d,grid.getLastDiffusion()); assert.ok(d.iterations<=4);
        const amplification=1/(1+options.kinematicViscosity*options.timeStep*lambda(c,1,1));
        for(let k=0;k<columns*rows;++k) {
            near(actual.xFaces[k],initial.xFaces[k]*amplification);
            near(actual.yFaces[k],initial.yFaces[k]*amplification);
        }
        assert.ok(d.gradientDissipation>0); assert.ok(d.finalKineticEnergy<d.initialKineticEnergy);
        grid.delete();
    }
    const c={columns:17,rows:12,spacingX:0.17,spacingY:0.31}, n=c.columns*c.rows;
    const grid=new physics.PeriodicMacGrid(c), a=mode(c,1,2,0.7,-0.4), b=mode(c,3,1,-0.2,0.5);
    const initial={xFaces:a.xFaces.map((v,k)=>v+b.xFaces[k]+0.17),yFaces:a.yFaces.map((v,k)=>v+b.yFaces[k]-0.23)};
    const mixedOptions={...options,kinematicViscosity:0.2,timeStep:0.031,density:7};
    grid.setVelocities(initial.xFaces,initial.yFaces);
    const d=grid.diffuse(mixedOptions), actual=grid.getVelocities(); auditDiagnostics(d);
    const fa=1/(1+0.2*0.031*lambda(c,1,2)), fb=1/(1+0.2*0.031*lambda(c,3,1));
    for(let k=0;k<n;++k) {
        near(actual.xFaces[k],fa*a.xFaces[k]+fb*b.xFaces[k]+0.17,1e-13);
        near(actual.yFaces[k],fa*a.yFaces[k]+fb*b.yFaces[k]-0.23,1e-13);
    }
    let oldSquare=0,newSquare=0,incrementSquare=0,gradientSquare=0,residualSquare=0,residualPairing=0;
    for(const component of ["xFaces","yFaces"]) for(let j=0;j<c.rows;++j) for(let i=0;i<c.columns;++i) {
        const k=i+c.columns*j, next=actual[component], old=initial[component];
        const dx=(next[k]-next[(i+1)%c.columns+c.columns*j])/c.spacingX;
        const dy=(next[k]-next[i+c.columns*((j+1)%c.rows)])/c.spacingY;
        const equation=(next[k]-old[k])+0.2*0.031*((next[k]-next[(i+1)%c.columns+c.columns*j]
            +next[k]-next[(i+c.columns-1)%c.columns+c.columns*j])/(c.spacingX*c.spacingX)
            +(next[k]-next[i+c.columns*((j+1)%c.rows)]+next[k]-next[i+c.columns*((j+c.rows-1)%c.rows)])/(c.spacingY*c.spacingY));
        oldSquare+=old[k]*old[k]; newSquare+=next[k]*next[k]; incrementSquare+=(next[k]-old[k])**2;
        gradientSquare+=dx*dx+dy*dy; residualSquare+=equation*equation; residualPairing+=next[k]*equation;
    }
    const mass=mixedOptions.density*c.spacingX*c.spacingY;
    near(d.initialKineticEnergy,0.5*mass*oldSquare,2e-13); near(d.finalKineticEnergy,0.5*mass*newSquare,2e-13);
    near(d.incrementKineticEnergy,0.5*mass*incrementSquare,1e-13);
    near(d.gradientDissipation,mass*0.2*0.031*gradientSquare,2e-13);
    near(d.finalResidualRms,Math.sqrt(residualSquare/n),2e-16); near(d.residualWork,mass*residualPairing,2e-13);
    near(d.finalMeanX,0.17); near(d.finalMeanY,-0.23);
    assert.ok(Math.abs(d.finalMeanX-d.initialMeanX)<=d.meanRoundoffAllowanceX);
    assert.ok(Math.abs(d.finalMeanY-d.initialMeanY)<=d.meanRoundoffAllowanceY);
    assert.ok(Math.abs(d.storageEnergyError)<=d.roundoffEnergyAllowance);
    assert.ok(Math.abs(d.residualWork)<=d.residualEnergyBound+d.roundoffEnergyAllowance);
    assert.ok(d.finalKineticEnergy<=d.initialKineticEnergy+d.residualEnergyBound+d.roundoffEnergyAllowance);

    const retained=grid.getLastDiffusion(), retainedCopy={...retained};
    d.finalKineticEnergy=99; assert.deepEqual(grid.getLastDiffusion(),retainedCopy);
    grid.project(); assert.deepEqual(grid.getLastDiffusion(),retainedCopy);
    const projection=grid.getLastProjection(); grid.setVelocities(initial.xFaces,initial.yFaces);
    assert.deepEqual(grid.getLastDiffusion(),retainedCopy);
    const sufficient=grid.diffuse(mixedOptions); grid.setVelocities(initial.xFaces,initial.yFaces);
    assert.deepEqual(grid.diffuse({...mixedOptions,maximumCellVisits:sufficient.cellVisits}),sufficient);
    assert.deepEqual(grid.getLastProjection(),projection);
    grid.setVelocities(initial.xFaces,initial.yFaces);
    const beforeV=grid.getVelocities(),beforeD=grid.getLastDiffusion();
    const unchanged=()=>{assert.deepEqual(grid.getVelocities(),beforeV);assert.deepEqual(grid.getLastDiffusion(),beforeD);assert.deepEqual(grid.getLastProjection(),projection);};
    for(const fail of [{...mixedOptions,maximumIterations:0},{...mixedOptions,maximumIterations:1},
        {...mixedOptions,maximumCellVisits:0},{...mixedOptions,maximumCellVisits:sufficient.cellVisits-1},
        {...mixedOptions,kinematicViscosity:1e308,timeStep:1e308},{...mixedOptions,density:1e308}]) {
        assert.throws(()=>grid.diffuse(fail)); unchanged();
    }
    for(const field of ["maximumIterations","maximumCellVisits"]) for(const bad of [-1,0.5,NaN,Infinity,-Infinity,2**32,2**32+1,Number.MAX_SAFE_INTEGER,
        field==="maximumIterations"?1000001:1000000001]) {
        assert.throws(()=>grid.diffuse({...mixedOptions,[field]:bad})); unchanged();
    }
    for(const field of ["kinematicViscosity","timeStep","density","absoluteVelocityTolerance","relativeVelocityTolerance"])
        for(const bad of [-1,NaN,Infinity,-Infinity]) { assert.throws(()=>grid.diffuse({...mixedOptions,[field]:bad})); unchanged(); }
    assert.throws(()=>grid.diffuse({...mixedOptions,density:0})); unchanged();
    for(const field of Object.keys(options)) { const incomplete={...options};delete incomplete[field];assert.throws(()=>grid.diffuse(incomplete));unchanged(); }

    const defaults=grid.diffuse(); auditDiagnostics(defaults);
    assert.equal(defaults.zeroTransportNoOp,true); assert.equal(defaults.kinematicViscosity,0); assert.equal(defaults.timeStep,0); assert.equal(defaults.density,1);
    assert.equal(defaults.targetResidualRms,1e-10); assert.equal(defaults.finalResidualRms,0); assert.equal(defaults.iterations,0);
    assert.equal(defaults.gradientDissipation,0); assert.equal(defaults.incrementKineticEnergy,0); assert.deepEqual(grid.getVelocities(),beforeV);
    for(const zero of [{kinematicViscosity:0,timeStep:1},{kinematicViscosity:1,timeStep:0}]) {
        const noop=grid.diffuse({...options,...zero,maximumIterations:0}); assert.equal(noop.zeroTransportNoOp,true);
        assert.deepEqual(grid.getVelocities(),beforeV); assert.equal(noop.initialKineticEnergy,noop.finalKineticEnergy);
    }
    const constant={xFaces:Array(n).fill(0.7),yFaces:Array(n).fill(-0.2)};grid.setVelocities(constant.xFaces,constant.yFaces);
    const constantD=grid.diffuse({...options,maximumIterations:0,maximumCellVisits:1000000000});
    assert.equal(constantD.iterations,0);assert.equal(constantD.zeroTransportNoOp,false);assert.equal(constantD.gradientDissipation,0);
    assert.deepEqual(grid.getVelocities(),constant);
    const saved=grid.getLastDiffusion(), savedCopy={...saved};grid.diffuse();assert.deepEqual(saved,savedCopy);
    saved.cellVisits=99;assert.notEqual(grid.getLastDiffusion().cellVisits,99);
    grid.delete();assert.deepEqual(retained,retainedCopy);assert.equal(saved.finalMeanX,constantD.finalMeanX);

    // One total iteration cannot pay for two independent single-mode solves.
    const shared=new physics.PeriodicMacGrid(c), single=mode(c,1,1,0.7,-0.4);
    shared.setVelocities(single.xFaces,single.yFaces);const sharedBefore=shared.getLastDiffusion();
    assert.throws(()=>shared.diffuse({...options,maximumIterations:1}));
    assert.deepEqual(shared.getVelocities(),single);assert.deepEqual(shared.getLastDiffusion(),sharedBefore);shared.delete();
    const zeroGrid=new physics.PeriodicMacGrid();const zeroD=zeroGrid.diffuse();assert.equal(zeroD.roundoffEnergyAllowance,0);assert.equal(zeroD.initialKineticEnergy,0);zeroGrid.delete();
}

function testThermalRadiation(physics) {
    const defaults = new physics.ThermalNetwork();
    const config = {...defaults.getConfig(), maxSubstep: 1};
    defaults.delete();
    const pair = new physics.ThermalNetwork(config);
    pair.addNode(2, 1); pair.addNode(1, 2);
    for (const invalid of [-1, NaN, Infinity]) {
        assert.throws(() => pair.addRadiationLink(0, 1, invalid));
        assert.equal(pair.getRadiationLinkCount(), 0);
    }
    for (const index of [-1, .5, 2, 2**32, NaN, Infinity])
        assert.throws(() => pair.addRadiationLink(0, index, 1));
    assert.equal(pair.addRadiationLink(0, 1, 1/16), 0);
    assert.throws(() => pair.addRadiationLink(1, 0, 1));
    for (const index of [-1, .5, 1, 2**32, NaN, Infinity])
        assert.throws(() => pair.getRadiationLink(index));
    const link = pair.getRadiationLink(0);
    assert.deepEqual(link, {first: 0, second: 1, coefficient: 1/16});
    link.coefficient = 99;
    assert.equal(pair.getRadiationLink(0).coefficient, 1/16);
    pair.step(.125);
    assert.equal(pair.getNode(0).temperature, 2-15/128);
    assert.equal(pair.getNode(1).temperature, 1+15/256);
    assert.equal(pair.getDiagnostics().totalEnergy, 4);
    assert.equal(pair.getDiagnostics().lastRadiativeVisits, 14);
    pair.applyPower(0, -1000);
    const before = [pair.getNode(0), pair.getNode(1), pair.getDiagnostics()];
    assert.throws(() => pair.step(.125));
    assert.deepEqual([pair.getNode(0), pair.getNode(1), pair.getDiagnostics()], before);
    pair.step(0);
    assert.equal(pair.getNode(0).externalPower, -1000);
    assert.equal(pair.getDiagnostics().lastRadiativeVisits, 0);
    pair.setConfig({...config, maxLinks: 1});
    assert.throws(() => pair.addLink(0, 1, 1));
    pair.delete();
    assert.equal(link.coefficient, 99); // Snapshot survives owner deletion.

    let previousError;
    for (const h of [1/512, 1/1024, 1/2048]) {
        const n = new physics.ThermalNetwork({...config, maxSubstep: h});
        n.addNode(4, 2); n.addNode(0, 1, true); n.addRadiationLink(0, 1, .25);
        n.step(.25);
        const error = 4/Math.cbrt(7)-n.getNode(0).temperature;
        assert.ok(error > 0);
        if (previousError) assert.ok(previousError/error > 1.95 && previousError/error < 2.1);
        assert.ok(Math.abs(n.getDiagnostics().totalEnergy-8-n.getDiagnostics().totalReservoirHeat) < 3e-14);
        previousError = error;
        n.delete();
    }
    assert.ok(previousError < .002);
    const heated = new physics.ThermalNetwork({...config, maxSubstep: .125, maxSubsteps: 2});
    heated.addNode(0, 1); heated.addNode(0, 1, true); heated.addRadiationLink(0, 1, 1);
    heated.applyPower(0, 64);
    const staged = [heated.getNode(0), heated.getNode(1), heated.getDiagnostics()];
    assert.throws(() => heated.step(.25));
    assert.deepEqual([heated.getNode(0), heated.getNode(1), heated.getDiagnostics()], staged);
    heated.setConfig({...config, maxSubstep: .125}); heated.step(.25);
    assert.ok(heated.getDiagnostics().lastSubsteps > 2);
    assert.ok(heated.getNode(0).temperature >= 0 && heated.getNode(0).temperature < 8);
    heated.delete();
}

async function main() {
    const physics = await createPhysicsEngineModule();
    testGravity(physics);
    testWaves(physics);
    testMacProjection(physics);
    testMacDiffusion(physics);
    testScalarTransport(physics);
    testMaxwell(physics);
    require("./maxwell-ohmic-tests.cjs").smoke(physics);
    require("./maxwell-mean-tests.cjs").smoke(physics);
    require("./elastic-wave-tests.cjs").smoke(physics);
    require("./electrostatic-tests.cjs").smoke(physics);
    testQueries(physics);
    const integerEngine = new physics.Engine();
    const integerConfig = integerEngine.getSimulationConfig();
    for (const key of ["maxSubstepsPerAdvance", "solverIterations", "maximumCcdImpacts"])
        for (const value of [-1, 0, 0.5, 3.5, NaN, Infinity, 2**31, 2**32, 2**32 + 1]) {
            assert.throws(() => integerEngine.setSimulationConfig({...integerConfig, [key]: value}));
            assert.deepEqual(integerEngine.getSimulationConfig(), integerConfig);
        }
    const validCounts = {...integerConfig, maxSubstepsPerAdvance: 3, solverIterations: 7, maximumCcdImpacts: 9};
    integerEngine.setSimulationConfig(validCounts);
    assert.deepEqual(integerEngine.getSimulationConfig(), validCounts);
    integerEngine.delete();
    const indexedParticles = physics.createParticleSystem();
    indexedParticles.reserve(0); indexedParticles.reserve(2);
    indexedParticles.addParticle({x: 7, y: 0}, {x: 0, y: 0}, 2);
    for (const index of [-1, 0.5, NaN, Infinity, 1, 2**32, 2**32 + 1]) {
        assert.throws(() => indexedParticles.getParticlePosition(index));
        assert.throws(() => indexedParticles.getParticleVelocity(index));
        assert.throws(() => indexedParticles.applyForce(index, {x: 100, y: 0}));
        assert.throws(() => indexedParticles.removeParticle(index));
        assert.equal(indexedParticles.size(), 1);
    }
    for (const count of [-1, 0.5, NaN, Infinity, 2**32, 2**32 + 1])
        assert.throws(() => indexedParticles.reserve(count));
    indexedParticles.applyForce(0, {x: 2, y: 0}); indexedParticles.step(0.5);
    assert.ok(Math.abs(indexedParticles.getParticlePosition(0).x - 7.125) < 1e-6);
    assert.equal(indexedParticles.getParticleVelocity(0).x, 0.5);
    indexedParticles.removeParticle(0); assert.equal(indexedParticles.size(), 0);
    indexedParticles.delete();
    const soft = new physics.SoftBody();
    const softConfig = soft.getConfig();
    assert.equal(softConfig.maxParticles, 100000);
    softConfig.maxSubstep = 0.001;
    assert.equal(soft.getConfig().maxSubstep, 0.01);
    const oscillator = new physics.SoftBody(softConfig);
    assert.equal(oscillator.addParticle({x: 0, y: 0}, {x: 0, y: 0}, 1, true), 0);
    assert.equal(oscillator.addParticle({x: 1.2, y: 0}, {x: 0, y: 0}, 1, false), 1);
    assert.equal(oscillator.addSpring(0, 1, 1, 4), 0);
    oscillator.step(1);
    assert.ok(Math.abs(oscillator.getParticle(1).position.x - (1 + 0.2 * Math.cos(2))) < 2e-5);
    assert.ok(Math.abs(oscillator.getParticle(1).velocity.x + 0.4 * Math.sin(2)) < 2e-5);
    assert.equal(oscillator.getParticleCount(), 2);
    assert.equal(oscillator.getSpringCount(), 1);
    assert.equal(oscillator.getSpring(0).stiffness, 4);
    assert.ok(oscillator.getDiagnostics().lastSubsteps >= 1000);
    assert.ok(Math.abs(oscillator.getDiagnostics().kineticEnergy + oscillator.getDiagnostics().elasticEnergy - 0.08) < 2e-6);
    const softSnapshot = oscillator.getParticle(1);
    softSnapshot.position.x = 99; softSnapshot.force.x = 99;
    assert.notEqual(oscillator.getParticle(1).position.x, 99);
    assert.equal(oscillator.getAccumulatedForce(1).x, 0);
    const springCopy = oscillator.getSpring(0); springCopy.stiffness = 99;
    assert.equal(oscillator.getSpring(0).stiffness, 4);
    const softDiagnosticsCopy = oscillator.getDiagnostics(); softDiagnosticsCopy.totalMass = 99;
    assert.equal(oscillator.getDiagnostics().totalMass, 2);
    for (const index of [-1, 0.5, NaN, Infinity, 2, 2**32, 2**32 + 1]) {
        assert.throws(() => oscillator.getParticle(index));
        assert.throws(() => oscillator.getAccumulatedForce(index));
        assert.throws(() => oscillator.setParticleState(index, {x: 0, y: 0}, {x: 0, y: 0}));
        assert.throws(() => oscillator.setFixed(index, false));
        assert.throws(() => oscillator.applyImpulse(index, {x: 1, y: 0}));
        assert.throws(() => oscillator.applyForce(index, 1, 0));
        assert.throws(() => oscillator.clearForces(index));
        assert.throws(() => oscillator.addSpring(index, 1, 1, 1, 0));
        assert.throws(() => oscillator.addSpring(0, index, 1, 1));
    }
    for (const index of [-1, 0.5, NaN, Infinity, 1, 2**32]) assert.throws(() => oscillator.getSpring(index));
    for (const field of ["maxSubsteps", "maxParticles", "maxSprings"]) {
        for (const count of [-1, 0.5, NaN, Infinity, 0, 2**32, 2**32 + 1]) {
            const invalid = {...softConfig, [field]: count};
            assert.throws(() => oscillator.setConfig(invalid));
            assert.throws(() => new physics.SoftBody(invalid));
            assert.equal(oscillator.getConfig()[field], softConfig[field]);
        }
    }
    assert.throws(() => oscillator.setUniformAcceleration({x: NaN, y: 0}));
    assert.throws(() => oscillator.setParticleState(1, {x: NaN, y: 0}));
    const forceConfig = soft.getConfig(); forceConfig.maxSubsteps = 2;
    soft.setConfig(forceConfig);
    soft.addParticle({x: 0, y: 0}, {x: 0, y: 0}, 2, false);
    soft.applyForce(0, 4, 0); soft.step(0);
    assert.equal(soft.getAccumulatedForce(0).x, 4);
    const forceBefore = soft.getParticle(0), diagnosticsBefore = soft.getDiagnostics();
    for (const dt of [-1, NaN, Infinity, 0.5]) {
        assert.throws(() => soft.step(dt));
        assert.deepEqual(soft.getParticle(0), forceBefore);
        assert.deepEqual(soft.getDiagnostics(), diagnosticsBefore);
    }
    forceConfig.maxSubsteps = 100; soft.setConfig(forceConfig); soft.step(0.5);
    assert.ok(Math.abs(soft.getParticle(0).position.x - 0.25) < 1e-6);
    assert.ok(Math.abs(soft.getParticle(0).velocity.x - 1) < 1e-6);
    assert.equal(soft.getAccumulatedForce(0).x, 0);
    soft.applyImpulse(0, {x: 2, y: 0}); assert.equal(soft.getParticle(0).velocity.x, 2);
    soft.applyForce(0, {x: 3, y: 0}); soft.clearForces(0); assert.equal(soft.getAccumulatedForce(0).x, 0);
    soft.applyForce(0, 3, 0); soft.clearForces(); assert.equal(soft.getAccumulatedForce(0).x, 0);
    soft.setUniformAcceleration({x: 0, y: -2}); assert.equal(soft.getUniformAcceleration().y, -2);
    soft.setFixed(0, true); soft.setParticleState(0, {x: 1, y: 0});
    soft.step(0.1); assert.equal(soft.getParticle(0).position.x, 1);
    const retainedSoft = soft.getParticle(0), retainedSpring = oscillator.getSpring(0);
    soft.delete(); oscillator.delete();
    assert.equal(retainedSoft.position.x, 1); assert.equal(retainedSpring.first, 0);
    assert.equal(softSnapshot.position.x, 99); // Deep JS snapshot survives deletion.
    const boundedSoft = new physics.SoftBody({...softConfig, maxParticles: 2, maxSprings: 1});
    boundedSoft.addParticle({x: 0, y: 0}); boundedSoft.addParticle({x: 1, y: 0});
    boundedSoft.addSpring(0, 1, 1, 1);
    assert.throws(() => boundedSoft.addParticle({x: 2, y: 0}));
    assert.throws(() => boundedSoft.addSpring(1, 0, 1, 1));
    assert.throws(() => boundedSoft.setConfig({...softConfig, maxParticles: 1}));
    assert.equal(boundedSoft.getParticleCount(), 2); assert.equal(boundedSoft.getSpringCount(), 1);
    boundedSoft.setConfig({...softConfig, maxParticles: 3, maxSprings: 1});
    boundedSoft.addParticle({x: 2, y: 0}); assert.throws(() => boundedSoft.addSpring(1, 2, 1, 1));
    assert.equal(boundedSoft.getSpringCount(), 1);
    boundedSoft.delete();
    const thermal = new physics.ThermalNetwork();
    const thermalConfig = thermal.getConfig();
    assert.equal(thermalConfig.maxNodes, 100000);
    const conduction = new physics.ThermalNetwork(thermalConfig);
    assert.equal(conduction.addNode(400, 2), 0); assert.equal(conduction.addNode(300, 3, false), 1);
    assert.equal(conduction.addLink(0, 1, 1), 0);
    const energyBefore = conduction.getDiagnostics().totalEnergy;
    conduction.step(1);
    const heatA = conduction.getNode(0), heatB = conduction.getNode(1);
    assert.ok(heatA.temperature < 400 && heatA.temperature > heatB.temperature);
    assert.ok(heatB.temperature > 300);
    assert.ok(Math.abs(conduction.getDiagnostics().totalEnergy - energyBefore) < 1e-10);
    assert.equal(conduction.getNodeCount(), 2); assert.equal(conduction.getLinkCount(), 1);
    assert.equal(conduction.getLink(0).conductance, 1);
    const linkCopy = conduction.getLink(0); linkCopy.conductance = 99;
    assert.equal(conduction.getLink(0).conductance, 1);
    const thermalDiagnosticsCopy = conduction.getDiagnostics(); thermalDiagnosticsCopy.totalEnergy = 99;
    assert.ok(Math.abs(conduction.getDiagnostics().totalEnergy - energyBefore) < 1e-10);
    heatA.temperature = 99; assert.notEqual(conduction.getNode(0).temperature, 99);
    for (const index of [-1, 0.5, NaN, Infinity, 2, 2**32, 2**32 + 1]) {
        assert.throws(() => conduction.getNode(index));
        assert.throws(() => conduction.setTemperature(index, 300));
        assert.throws(() => conduction.setFixed(index, true));
        assert.throws(() => conduction.applyPower(index, 1));
        assert.throws(() => conduction.clearPowers(index));
        assert.throws(() => conduction.addLink(index, 1, 1));
        assert.throws(() => conduction.addLink(0, index, 1));
    }
    for (const index of [-1, 0.5, NaN, Infinity, 1, 2**32]) assert.throws(() => conduction.getLink(index));
    for (const field of ["maxSubsteps", "maxNodes", "maxLinks"]) {
        for (const count of [-1, 0.5, NaN, Infinity, 0, 2**32, 2**32 + 1]) {
            const invalid = {...thermalConfig, [field]: count};
            assert.throws(() => conduction.setConfig(invalid));
            assert.throws(() => new physics.ThermalNetwork(invalid));
            assert.equal(conduction.getConfig()[field], thermalConfig[field]);
        }
    }
    assert.throws(() => conduction.setTemperature(0, -1));
    assert.throws(() => conduction.addLink(1, 0, 1));
    assert.throws(() => conduction.applyPower(0, Infinity));
    assert.throws(() => new physics.ThermalNetwork({...thermalConfig, safetyFactor: 2}));
    thermal.addNode(300, 2); thermal.applyPower(0, 4); thermal.step(0);
    assert.equal(thermal.getNode(0).externalPower, 4);
    const heatingConfig = thermal.getConfig(); heatingConfig.maxSubsteps = 2; thermal.setConfig(heatingConfig);
    const heatingBefore = thermal.getNode(0), heatingDiagnostics = thermal.getDiagnostics();
    for (const dt of [-1, NaN, Infinity, 0.5]) {
        assert.throws(() => thermal.step(dt));
        assert.deepEqual(thermal.getNode(0), heatingBefore);
        assert.deepEqual(thermal.getDiagnostics(), heatingDiagnostics);
    }
    heatingConfig.maxSubsteps = 100; thermal.setConfig(heatingConfig); thermal.step(0.5);
    assert.ok(Math.abs(thermal.getNode(0).temperature - 301) < 1e-10);
    assert.equal(thermal.getNode(0).externalPower, 0);
    assert.ok(Math.abs(thermal.getDiagnostics().totalExternalEnergy - 2) < 1e-12);
    thermal.applyPower(0, -10000);
    const coolingBefore = thermal.getNode(0), coolingDiagnostics = thermal.getDiagnostics();
    assert.throws(() => thermal.step(0.5));
    assert.deepEqual(thermal.getNode(0), coolingBefore); assert.deepEqual(thermal.getDiagnostics(), coolingDiagnostics);
    thermal.clearPowers(0); assert.equal(thermal.getNode(0).externalPower, 0);
    thermal.applyPower(0, 1); thermal.clearPowers(); assert.equal(thermal.getNode(0).externalPower, 0);
    thermal.setFixed(0, true); thermal.applyPower(0, 4); thermal.step(0.5);
    assert.ok(Math.abs(thermal.getNode(0).temperature - 301) < 1e-10);
    assert.ok(Math.abs(thermal.getNode(0).reservoirHeat + 2) < 1e-12);
    assert.ok(Math.abs(thermal.getDiagnostics().totalReservoirHeat + 2) < 1e-12);
    thermal.setFixed(0, false); thermal.setTemperature(0, 350);
    const retainedNode = thermal.getNode(0), retainedLink = conduction.getLink(0);
    thermal.delete(); conduction.delete();
    assert.equal(retainedNode.temperature, 350); assert.equal(retainedLink.second, 1); assert.equal(heatA.temperature, 99);
    const boundedHeat = new physics.ThermalNetwork({...thermalConfig, maxNodes: 2, maxLinks: 1});
    boundedHeat.addNode(300, 1); boundedHeat.addNode(300, 1); boundedHeat.addLink(0, 1, 1);
    assert.throws(() => boundedHeat.addNode(300, 1));
    assert.throws(() => boundedHeat.setConfig({...thermalConfig, maxNodes: 1}));
    assert.equal(boundedHeat.getNodeCount(), 2); assert.equal(boundedHeat.getLinkCount(), 1);
    boundedHeat.setConfig({...thermalConfig, maxNodes: 3, maxLinks: 1});
    boundedHeat.addNode(300, 1); assert.throws(() => boundedHeat.addLink(1, 2, 1));
    assert.equal(boundedHeat.getLinkCount(), 1);
    boundedHeat.delete();
    testThermalRadiation(physics);
    const charge = new physics.ChargedParticle({x: 0, y: 0}, {x: 2, y: 0}, 3, 6);
    charge.step(Math.PI / 8, {electric: {x: 0, y: 0}, magnetic: 2});
    assert.ok(Math.abs(charge.getPosition().x - 0.5) < 1e-12);
    assert.ok(Math.abs(charge.getPosition().y + 0.5) < 1e-12);
    assert.ok(Math.abs(charge.getVelocity().y + 2) < 1e-12);
    assert.ok(Math.abs(charge.getKineticEnergy() - 6) < 1e-12);
    assert.equal(charge.getMass(), 3);
    assert.equal(charge.getCharge(), 6);
    const snapshot = charge.getPosition();
    snapshot.x = 99; // Observers return JS values, not borrowed mutable state.
    assert.ok(Math.abs(charge.getPosition().x - 0.5) < 1e-12);
    assert.throws(() => charge.setState({x: NaN, y: 0}, {x: 0, y: 0}));
    assert.throws(() => charge.step(-1, {electric: {x: 0, y: 0}, magnetic: 0}));
    charge.setState({x: 0, y: 0}, {x: 0, y: 0});
    charge.step(0.5, {electric: {x: 2, y: 0}, magnetic: 0});
    assert.ok(Math.abs(charge.getPosition().x - 0.5) < 1e-12);
    assert.ok(Math.abs(charge.getVelocity().x - 2) < 1e-12);
    charge.delete();
    const neutral = new physics.ChargedParticle();
    assert.equal(neutral.getCharge(), 0);
    neutral.delete();
    const engine = new physics.Engine();
    const simulationConfig = engine.getSimulationConfig();
    assert.equal(simulationConfig.solverIterations, 10);
    assert.ok(Math.abs(simulationConfig.fixedTimeStep - 1 / 60) < 0.000001);
    assert.equal(simulationConfig.maxSubstepsPerAdvance, 8);
    assert.equal(simulationConfig.restitutionVelocityThreshold, 1.0);
    assert.ok(Math.abs(simulationConfig.velocityTolerance - 0.0001) < 0.000001);
    assert.ok(Math.abs(simulationConfig.maxPositionCorrection - 0.2) < 0.000001);
    assert.equal(simulationConfig.enableLinearVelocityLimit, true);
    simulationConfig.solverIterations = 4;
    simulationConfig.restitutionVelocityThreshold = 0.75;
    simulationConfig.enableLinearVelocityLimit = false;
    simulationConfig.enableAngularVelocityLimit = false;
    engine.setSimulationConfig(simulationConfig);
    const configuredSimulation = engine.getSimulationConfig();
    assert.equal(configuredSimulation.solverIterations, 4);
    assert.equal(configuredSimulation.restitutionVelocityThreshold, 0.75);
    assert.equal(configuredSimulation.enableLinearVelocityLimit, false);
    assert.equal(configuredSimulation.enableAngularVelocityLimit, false);
    const shape = new physics.Circle(1.0);
    const body = physics.createRigidBody(
        shape,
        {
            density: 1.0,
            restitution: 0.5,
            staticFriction: 0.6,
            dynamicFriction: 0.4,
        },
        { x: 0.0, y: 0.0 },
        false,
    );

    // The body owns a copy: deleting the JavaScript shape must be safe.
    shape.delete();

    body.setCollisionCategoryBits(0x00000002);
    body.setCollisionMaskBits(0x00000004);
    assert.equal(body.getCollisionCategoryBits(), 0x00000002);
    assert.equal(body.getCollisionMaskBits(), 0x00000004);

    body.setVelocity({ x: 3.0, y: 0.0 });
    engine.addBody(body);
    engine.step(0.5);

    const position = body.getPosition();
    assert.ok(Math.abs(position.x - 1.5) < 0.0001, `unexpected x position: ${position.x}`);
    assert.equal(position.y, 0);

    const particles = physics.createParticleSystem();
    particles.addParticle({ x: 0.0, y: 0.0 }, { x: 4.0, y: 0.0 }, 1.0);
    engine.addParticleSystem(particles);
    engine.step(0.25);
    const particlePosition = particles.getParticlePosition(0);
    assert.ok(
        Math.abs(particlePosition.x - 1.0) < 0.0001,
        `unexpected particle x position: ${particlePosition.x}`,
    );

    simulationConfig.fixedTimeStep = 0.1;
    simulationConfig.maxSubstepsPerAdvance = 2;
    engine.setSimulationConfig(simulationConfig);
    engine.resetTiming();
    const fixedProgress = engine.advance(0.25);
    assert.equal(fixedProgress.stepsPerformed, 2);
    assert.ok(Math.abs(fixedProgress.simulatedTime - 0.2) < 0.000001);
    assert.ok(Math.abs(fixedProgress.remainingTime - 0.05) < 0.000001);
    assert.equal(engine.getTotalStepCount(), 2n);

    const caughtUp = engine.advance(0.05);
    assert.equal(caughtUp.stepsPerformed, 1);
    assert.ok(engine.getAccumulatedTime() < 0.000001);
    const statistics = engine.getLastStepStatistics();
    assert.equal(statistics.integratedBodyCount, 1);
    assert.equal(statistics.integratedParticleCount, 1);
    assert.equal(statistics.broadPhaseCandidateCount, 0);
    assert.equal(statistics.resolvedContactCount, 0);
    assert.equal(statistics.solverIterationCount, 4);
    assert.equal(statistics.fluidIterationCount, 0);

    const exported = JSON.parse(engine.exportJson(1));
    assert.equal(exported.schemaVersion, 1);
    assert.equal(exported.bodies.length, 1);
    assert.equal(exported.bodies[0].shape.type, "circle");
    assert.equal(typeof exported.bodies[0].id, "string");
    assert.ok(engine.exportCsv(1).startsWith("time,id,"));
    assert.throws(() => body.setVelocity({ x: NaN, y: 0 }));
    const box = physics.Polygon.makeBox(1, 1);
    const second = physics.createRigidBody(box, {density: 1, restitution: 0,
        staticFriction: 0, dynamicFriction: 0}, {x: 8, y: 0}, false);
    box.delete();
    second.setCollisionMaskBits(0);
    engine.addBody(second);
    const joint = physics.createDistanceJoint(body, second, 2, {x: 0, y: 0}, {x: 0, y: 0});
    engine.addJoint(joint);
    // World and joint retain their shared bodies after JS handles are deleted.
    second.delete(); body.delete(); joint.delete();
    for (let i = 0; i < 20; ++i) engine.stepFixed();
    assert.equal(JSON.parse(engine.exportJson(2)).bodies.length, 2);
    engine.clearBodies();
    assert.equal(JSON.parse(engine.exportJson(2)).bodies.length, 0);
    const rotorShape = new physics.Circle(1);
    const material = {density: 1, restitution: 0, staticFriction: 0, dynamicFriction: 0};
    const support = physics.createRigidBody(rotorShape, material, {x: 0, y: 0}, true);
    const rotor = physics.createRigidBody(rotorShape, material, {x: 0, y: 0}, false);
    rotorShape.delete();
    rotor.setMass(2); // Unit-radius disk: I = m*r^2/2 = 1.
    rotor.setCollisionMaskBits(0);
    engine.addBody(support); engine.addBody(rotor);
    const hinge = physics.createRevoluteJoint(support, rotor, {x: 0, y: 0}, {x: 0, y: 0});
    assert.equal(hinge.isMotorEnabled(), false);
    assert.equal(hinge.areLimitsEnabled(), false);
    hinge.setMotor(true, 10, 2);
    assert.equal(hinge.getMotorSpeed(), 10);
    assert.equal(hinge.getMaxMotorTorque(), 2);
    engine.addJoint(hinge);
    engine.step(0.25);
    assert.ok(Math.abs(rotor.getAngularVelocity() - 0.5) < 1e-6);
    assert.ok(Math.abs(hinge.getMotorTorque() - 2) < 1e-6);
    engine.step(0);
    assert.equal(hinge.getMotorTorque(), 0);
    hinge.setLimits(true, -0.1, 0.2);
    assert.equal(hinge.areLimitsEnabled(), true);
    assert.ok(Math.abs(hinge.getLowerLimit() + 0.1) < 1e-6);
    assert.ok(Math.abs(hinge.getUpperLimit() - 0.2) < 1e-6);
    assert.throws(() => hinge.setLimits(true, 2, 1));
    assert.throws(() => hinge.setMotor(true, NaN, 1));
    support.delete(); rotor.delete();
    for (let i = 0; i < 300; ++i) engine.step(1 / 120);
    assert.ok(Math.abs(hinge.getAngle() - 0.2) < 1e-5);
    assert.ok(Math.abs(hinge.getAnchorA().x - hinge.getAnchorB().x) < 1e-6);
    engine.removeJoint(hinge);
    hinge.delete();
    engine.clearBodies();
    const sliderShape = new physics.Circle(0.1);
    const rail = physics.createRigidBody(sliderShape, material, {x: 0, y: 0}, true);
    const carriage = physics.createRigidBody(sliderShape, material, {x: 0, y: 0}, false);
    sliderShape.delete();
    carriage.setMass(2);
    carriage.setCollisionMaskBits(0);
    engine.addBody(rail); engine.addBody(carriage);
    const slider = physics.createPrismaticJoint(rail, carriage, {x: 1, y: 0}, {x: 0, y: 0}, {x: 0, y: 0});
    assert.equal(slider.isMotorEnabled(), false);
    assert.equal(slider.areLimitsEnabled(), false);
    slider.setMotor(true, 10, 4);
    engine.addJoint(slider);
    engine.step(0.25);
    assert.ok(Math.abs(slider.getTranslationSpeed() - 0.5) < 1e-6);
    assert.ok(Math.abs(slider.getMotorForce() - 4) < 1e-6);
    slider.setLimits(true, -0.1, 0.2);
    assert.throws(() => slider.setLimits(true, 2, 1));
    assert.throws(() => slider.setMotor(true, 1, -1));
    assert.equal(slider.getMaxMotorForce(), 4);
    assert.equal(slider.getMotorSpeed(), 10);
    assert.equal(slider.areLimitsEnabled(), true);
    rail.delete(); carriage.delete();
    for (let i = 0; i < 300; ++i) engine.step(1 / 120);
    assert.ok(Math.abs(slider.getTranslation() - 0.2) < 1e-5);
    assert.ok(Math.abs(slider.getTransverseError()) < 1e-6);
    assert.ok(Math.abs(slider.getAngle()) < 1e-6);
    engine.step(0);
    assert.equal(slider.getMotorForce(), 0);
    engine.clearBodies();
    assert.ok(Math.abs(slider.getTranslation() - 0.2) < 1e-5); // Joint retains both endpoints.
    slider.delete();
    engine.delete();
    particles.delete();
    require("./handle-lifetime-tests.cjs").smoke(physics);
    console.log("PASS: configuration, stepping, filtering, lifetimes, owned spatial queries, joint motors/limits, exports, particles, electromagnetic motion, soft-body oscillator/loads, thermal conservation/accounting, N-body gravity, membrane waves, periodic scalar transport, periodic MAC projection/diffusion, periodic TMz Maxwell fields, elastic waves and static electrostatics");
}

main().catch((error) => {
    console.error(error);
    process.exitCode = 1;
});
