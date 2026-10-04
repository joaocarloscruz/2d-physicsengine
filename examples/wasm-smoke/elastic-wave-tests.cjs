const assert = require('node:assert/strict');
const fields = ['vx', 'vy', 'sigmaXX', 'sigmaYY', 'sigmaXY'];
const offsets = [[0,.5],[.5,0],[.5,.5],[.5,.5],[0,0]];
const zero = n => Object.fromEntries(fields.map(f => [f, Array(n).fill(0)]));
const snapshot = g => ({state:g.getState(), diagnostics:g.getDiagnostics()});
function close(a,b,tol,label='') {
    assert.ok(Math.abs(a-b) <= tol, `${label}: ${a} != ${b}, absolute margin ${tol}`);
}
function compare(actual, expected, tol) {
    for (const f of fields) for (let k=0;k<actual[f].length;++k)
        close(actual[f][k],expected[f][k],tol,`${f}[${k}]`);
}
function sample(c,mx,my,real,imag=Array(5).fill(0)) {
    const s=zero(c.columns*c.rows);
    for(let j=0;j<c.rows;++j) for(let i=0;i<c.columns;++i) for(let f=0;f<5;++f) {
        const phase=2*Math.PI*(mx*(i+offsets[f][0])/c.columns+my*(j+offsets[f][1])/c.rows)+.31;
        s[fields[f]][i+c.columns*j]=real[f]*Math.cos(phase)-imag[f]*Math.sin(phase);
    }
    return s;
}
// Closed analytic mode of the stress-kick/velocity-drift/stress-kick map.
// This does not iterate its finite-difference implementation.
function mode(c,mx,my,shear,h,steps,continuousTime) {
    const continuum=continuousTime!==undefined;
    const x=continuum?2*Math.PI*mx/(c.columns*c.spacingX):2*Math.sin(Math.PI*mx/c.columns)/c.spacingX;
    const y=continuum?2*Math.PI*my/(c.rows*c.spacingY):2*Math.sin(Math.PI*my/c.rows)/c.spacingY;
    const k=Math.hypot(x,y), speed=Math.sqrt((shear?c.shearModulus:c.lambda+2*c.shearModulus)/c.density);
    const omega=speed*k, dx=shear?-y/k:x/k, dy=shear?x/k:y/k;
    const angle=continuum?omega*continuousTime:steps*2*Math.asin(h*omega/2);
    const stressScale=Math.sin(angle)/omega*(continuum?1:Math.sqrt(1-(h*omega/2)**2));
    return sample(c,mx,my,[dx*Math.cos(angle),dy*Math.cos(angle),0,0,0],
        [0,0,((c.lambda+2*c.shearModulus)*x*dx+c.lambda*y*dy)*stressScale,
        (c.lambda*x*dx+(c.lambda+2*c.shearModulus)*y*dy)*stressScale,
        c.shearModulus*(y*dx+x*dy)*stressScale]);
}
// Independently assembled periodic incidence and constitutive power pairing.
function rates(c,s) {
    const n=c.columns*c.rows, r={accelerationX:Array(n),accelerationY:Array(n),
        strainRateXX:Array(n),strainRateYY:Array(n),engineeringShearRate:Array(n)};
    const at=(i,j)=>(i+c.columns)%c.columns+c.columns*((j+c.rows)%c.rows);
    for(let j=0;j<c.rows;++j) for(let i=0;i<c.columns;++i) {
        const k=at(i,j),xp=at(i+1,j),xm=at(i-1,j),yp=at(i,j+1),ym=at(i,j-1);
        r.strainRateXX[k]=(s.vx[xp]-s.vx[k])/c.spacingX;
        r.strainRateYY[k]=(s.vy[yp]-s.vy[k])/c.spacingY;
        r.engineeringShearRate[k]=(s.vx[k]-s.vx[ym])/c.spacingY+(s.vy[k]-s.vy[xm])/c.spacingX;
        r.accelerationX[k]=((s.sigmaXX[k]-s.sigmaXX[xm])/c.spacingX+(s.sigmaXY[yp]-s.sigmaXY[k])/c.spacingY)/c.density;
        r.accelerationY[k]=((s.sigmaXY[xp]-s.sigmaXY[k])/c.spacingX+(s.sigmaYY[k]-s.sigmaYY[ym])/c.spacingY)/c.density;
    }
    return r;
}
function energies(c,s,h) {
    let kinetic=0,strain=0,correction=0;
    const r=rates(c,s),V=c.spacingX*c.spacingY;
    for(let k=0;k<s.vx.length;++k) {
        kinetic+=c.density*(s.vx[k]**2+s.vy[k]**2)/2;
        strain+=(s.sigmaXX[k]+s.sigmaYY[k])**2/(8*(c.lambda+c.shearModulus))
            +(s.sigmaXX[k]-s.sigmaYY[k])**2/(8*c.shearModulus)+s.sigmaXY[k]**2/(2*c.shearModulus);
        correction+=(c.lambda+c.shearModulus)*(r.strainRateXX[k]+r.strainRateYY[k])**2
            +c.shearModulus*(r.strainRateXX[k]-r.strainRateYY[k])**2
            +c.shearModulus*r.engineeringShearRate[k]**2;
    }
    return {kinetic:V*kinetic,strain:V*strain,modified:V*(kinetic+strain-h*h*correction/8)};
}
function smoke(p) {
    assert.equal(typeof p.ElasticWaveGrid,'function');
    const defaults=new p.ElasticWaveGrid(),base=defaults.getConfig();
    assert.deepEqual(base,{columns:16,rows:16,spacingX:1,spacingY:1,density:1,lambda:1,shearModulus:1,
        cflSafety:.9,maxSubstep:.1,maximumSubsteps:10000,maximumCellVisits:100000000});
    defaults.delete();
    let modes=0;
    for(const [nx,ny,mx,my] of [[12,10,1,0],[12,10,0,2],[12,10,2,3],[2,5,1,2],[5,2,2,1],[2,2,1,1]])
        for(const shear of [false,true]) for(const lambda of [3,-.5]) {
            const c={...base,columns:nx,rows:ny,spacingX:.23,spacingY:.41,density:2,lambda,shearModulus:1,maxSubstep:1};
            const g=new p.ElasticWaveGrid(c),h=.7*g.getStableTimeStep();
            const initial=mode(c,mx,my,shear,h,0);g.setState(initial);
            close(g.getCompressionalSpeed(),Math.sqrt((lambda+2)/2),1e-15);
            close(g.getShearSpeed(),Math.sqrt(.5),1e-15);
            const reference=energies(c,initial,h).modified;
            close(g.getModifiedEnergy(h),reference,2e-13);
            for(let step=0;step<71;++step) g.step(h);
            compare(g.getState(),mode(c,mx,my,shear,h,71),3e-12);
            const d=g.getDiagnostics(),expected=energies(c,g.getState(),h);
            assert.equal(d.lastSubsteps,1);assert.equal(d.lastCellVisits,4*nx*ny);close(d.time,71*h,2e-14);
            close(d.kineticEnergy,expected.kinetic,3e-12);close(d.strainEnergy,expected.strain,3e-12);
            close(d.modifiedEnergy,reference,3e-12);
            assert.ok(d.totalEnergy>=reference-3e-12&&d.totalEnergy<=d.physicalEnergyUpperBound+3e-12);
            close(d.physicalEnergyUpperBound,reference/(1-(h*g.getCompressionalSpeed()*Math.hypot(1/c.spacingX,1/c.spacingY))**2),1e-11);
            assert.ok(d.maxAbsCompatibility<2e-10);close(d.meanVx,0,2e-14);close(d.meanVy,0,2e-14);
            g.delete();++modes;
        }
    for(const shear of [false,true]) for(const [mx,my] of [[1,0],[0,1],[1,2]]) {
        let previous;
        for(const n of [16,32,64]) {
            const c={...base,columns:n,rows:n,spacingX:2/n,spacingY:3/n,density:2,lambda:3,shearModulus:2,maxSubstep:1};
            const g=new p.ElasticWaveGrid(c),T=.19,steps=Math.ceil(T/(.4*g.getStableTimeStep())),h=T/steps;
            g.setState(mode(c,mx,my,shear,0,0,0));for(let k=0;k<steps;++k) g.step(h);
            const actual=g.getState(),expected=mode(c,mx,my,shear,0,0,T);
            let error=0;for(const f of fields) for(let k=0;k<n*n;++k) error=Math.max(error,Math.abs(actual[f][k]-expected[f][k]));
            if(previous!==undefined) assert.ok(error/previous>.18&&error/previous<.31,`continuum P/S ratio ${error/previous}`);
            previous=error;g.delete();
        }
    }
    const c={...base,columns:9,rows:7,spacingX:.2,spacingY:.35,density:4,lambda:2,shearModulus:3,maxSubstep:1};
    const g=new p.ElasticWaveGrid(c),s=zero(63);
    for(let f=0;f<5;++f) for(let k=0;k<63;++k) s[fields[f]][k]=Math.sin(.43*(k+1)*(f+1))+.07*f;
    g.setState(s);const before=snapshot(g),defect=g.getCompatibility(),zz=g.getOutOfPlaneStress();
    const independentRates=rates(c,s),actualRates=g.getSpatialRates();
    let power=0;
    for(let k=0;k<63;++k) {
        for(const f of Object.keys(independentRates)) close(actualRates[f][k],independentRates[f][k],2e-14);
        power+=s.sigmaXX[k]*independentRates.strainRateXX[k]+s.sigmaYY[k]*independentRates.strainRateYY[k]
            +s.sigmaXY[k]*independentRates.engineeringShearRate[k]+c.density*(s.vx[k]*independentRates.accelerationX[k]+s.vy[k]*independentRates.accelerationY[k]);
        close(zz[k],c.lambda/(2*(c.lambda+c.shearModulus))*(s.sigmaXX[k]+s.sigmaYY[k]),2e-16);
    }
    close(power,0,2e-13,'adjoint power');
    const h=.7*g.getStableTimeStep(),reference=energies(c,s,h).modified;
    for(let k=0;k<400;++k) {
        g.step(h);const d=g.getDiagnostics();close(d.modifiedEnergy,reference,2e-11);
        assert.ok(d.totalEnergy>=reference-2e-11&&d.totalEnergy<=d.physicalEnergyUpperBound+2e-11);
    }
    const after=g.getDiagnostics();
    for(const f of ['meanVx','meanVy','meanSigmaXX','meanSigmaYY','meanSigmaXY','meanSigmaZZ']) close(after[f],before.diagnostics[f],3e-14);
    const finalDefect=g.getCompatibility();for(let k=0;k<63;++k) close(finalDefect[k],defect[k],2e-11);
    // Every observation and supplied array is copied; no JS/native field aliases.
    const saved=snapshot(g),owned=g.getState();for(const f of fields) owned[f][0]=100;
    const configCopy=g.getConfig();configCopy.columns=100;
    const ratesCopy=g.getSpatialRates();for(const a of Object.values(ratesCopy)) a[0]=100;
    const diagCopy=g.getDiagnostics();diagCopy.time=100;
    const compatibilityCopy=g.getCompatibility();compatibilityCopy[0]=100;
    const zzCopy=g.getOutOfPlaneStress();zzCopy[0]=100;
    assert.deepEqual(snapshot(g),saved);assert.equal(g.getConfig().columns,9);
    g.step(0);assert.deepEqual(snapshot(g),saved);
    for(const bad of [-1,NaN,Infinity,Number.MIN_VALUE]) {assert.throws(()=>g.step(bad));assert.deepEqual(snapshot(g),saved);}
    for(const bad of [-1,NaN,Infinity,1]) assert.throws(()=>g.getModifiedEnergy(bad));
    for(const field of fields) for(const bad of [[],Array(63),Array(63).fill(Infinity),Array(63).fill('1'),new Float64Array(63)]) {
        const malformed={...s,[field]:bad};assert.throws(()=>g.setState(malformed));assert.deepEqual(snapshot(g),saved);
    }
    const overflow=zero(63);overflow.vx.fill(1e308);assert.throws(()=>g.setState(overflow));assert.deepEqual(snapshot(g),saved);
    const tiny=zero(63);tiny.vx.fill(1e-300);assert.throws(()=>g.setState(tiny));assert.deepEqual(snapshot(g),saved);
    const mixed=zero(63);mixed.vx[0]=1e100;mixed.vx[1]=1e-300;mixed.vx[2]=-1e100;
    assert.throws(()=>g.setState(mixed));assert.deepEqual(snapshot(g),saved);
    const a=new p.ElasticWaveGrid(c),b=new p.ElasticWaveGrid(c);a.setState(s);b.setState(s);
    // Caller edits after publishing cannot affect either owner.
    s.vx[0]=123;for(const dt of [.021,0,.137,.004]) {a.step(dt);b.step(dt);}
    assert.deepEqual(snapshot(a),snapshot(b));const time=a.getDiagnostics().time;
    a.setState(a.getState());assert.equal(a.getDiagnostics().time,time);assert.equal(a.getDiagnostics().lastSubsteps,0);
    a.delete();b.delete();g.delete();
    const dc=new p.ElasticWaveGrid({...base,columns:2,rows:3,spacingX:.2,spacingY:.4,density:4,lambda:2,shearModulus:3});
    const constant=zero(6);for(const [f,v] of Object.entries({vx:2,vy:-3,sigmaXX:4,sigmaYY:5,sigmaXY:6})) constant[f].fill(v);
    dc.setState(constant);dc.step(.037);assert.deepEqual(dc.getState(),constant);
    const dcConfig=dc.getConfig(),V=dcConfig.spacingX*dcConfig.spacingY,nDC=constant.vx.length;
    const dcKinetic=V*nDC*dcConfig.density*(2**2+(-3)**2)/2;
    const dcStrain=V*nDC*((4+5)**2/(8*(dcConfig.lambda+dcConfig.shearModulus))
        +(4-5)**2/(8*dcConfig.shearModulus)+6**2/(2*dcConfig.shearModulus));
    // Scale solely with the represented physical energy. The conservative
    // operation budget covers weights/products, 2 or 3 hypot terms per cell,
    // and final squaring, including compounded norm-to-energy rounding.
    const roundoff=ops=>ops*Number.EPSILON/(1-ops*Number.EPSILON);
    close(dc.getDiagnostics().kineticEnergy,dcKinetic,roundoff(16*nDC+32)*dcKinetic);
    close(dc.getDiagnostics().strainEnergy,dcStrain,roundoff(24*nDC+48)*dcStrain);
    for(const z of dc.getOutOfPlaneStress()) close(z,1.8,3e-16);dc.delete();
    // Independent compression/shear responses, including engineering shear convention.
    for(const shear of [false,true]) {
        const x=new p.ElasticWaveGrid({...base,columns:4,rows:2,lambda:2,shearModulus:3});
        const t=zero(8);t[shear?'vy':'vx']=[0,1,0,-1,0,1,0,-1];x.setState(t);x.step(.01);const out=x.getState();
        if(shear) {assert.deepEqual(out.sigmaXX,Array(8).fill(0));assert.deepEqual(out.sigmaYY,Array(8).fill(0));assert.ok(out.sigmaXY.some(v=>v!==0));}
        else {assert.deepEqual(out.sigmaXY,Array(8).fill(0));for(let k=0;k<8;++k) close(out.sigmaYY[k],.25*out.sigmaXX[k],2e-17);}
        x.delete();
    }
    for(const f of ['columns','rows','maximumSubsteps','maximumCellVisits']) for(const bad of [-1,0,.5,NaN,Infinity,2**32,2**32+1,Number.MAX_SAFE_INTEGER])
        assert.throws(()=>new p.ElasticWaveGrid({...base,[f]:bad}));
    assert.throws(()=>new p.ElasticWaveGrid({...base,columns:512,rows:513}));
    for(const f of ['density','shearModulus','spacingX','spacingY','maxSubstep']) for(const bad of [-1,0,NaN,Infinity])
        assert.throws(()=>new p.ElasticWaveGrid({...base,[f]:bad}));
    for(const bad of [-.8,NaN,Infinity]) assert.throws(()=>new p.ElasticWaveGrid({...base,lambda:bad}));
    for(const bad of [0,-1,1,NaN,Infinity]) assert.throws(()=>new p.ElasticWaveGrid({...base,cflSafety:bad}));
    assert.throws(()=>new p.ElasticWaveGrid({...base,spacingX:1e200}));
    assert.throws(()=>new p.ElasticWaveGrid({...base,maximumSubsteps:1000001}));
    assert.throws(()=>new p.ElasticWaveGrid({...base,maximumCellVisits:1000000001}));
    const budget=new p.ElasticWaveGrid({...base,columns:2,rows:2,maximumCellVisits:15});
    const budgetBefore=snapshot(budget);assert.throws(()=>budget.step(.01));assert.deepEqual(snapshot(budget),budgetBefore);budget.delete();
    const count=new p.ElasticWaveGrid({...base,columns:2,rows:2,maxSubstep:.01,maximumSubsteps:1});
    const countBefore=snapshot(count);assert.throws(()=>count.step(.02));assert.deepEqual(snapshot(count),countBefore);count.delete();
    const late=new p.ElasticWaveGrid({...base,columns:2,rows:2,maxSubstep:1});
    const large=zero(4);large.sigmaXX=[1.4e154,-1.4e154,1.4e154,-1.4e154];late.setState(large);
    const lateBefore=snapshot(late);assert.throws(()=>late.step(.35));assert.deepEqual(snapshot(late),lateBefore);late.delete();
    const extreme=new p.ElasticWaveGrid({...base,columns:2,rows:2,spacingX:1e-8,spacingY:1e-8,density:1e300,lambda:-.5e300,shearModulus:1e300});
    const opposite=zero(4);opposite.sigmaXX=[1e308,-1e308,1e308,-1e308];extreme.setState(opposite);
    const extremeBefore=snapshot(extreme);assert.throws(()=>extreme.step(1e-10));assert.deepEqual(snapshot(extreme),extremeBefore);extreme.delete();
    const clock=new p.ElasticWaveGrid({...base,columns:2,rows:2,lambda:1e-40,shearModulus:1e-40,maxSubstep:1e14});
    clock.step(1e14);const clockBefore=snapshot(clock);assert.throws(()=>clock.step(1e-10));assert.deepEqual(snapshot(clock),clockBefore);clock.delete();
    // Aggregate energies are representable even though every squared sample
    // and every per-sample contribution to the old mean would underflow.
    const subConfig={...base,columns:512,rows:512,density:1e308},n=512*512,value=7e-319;
    const sub=new p.ElasticWaveGrid(subConfig),subState=zero(n);subState.vx.fill(value);
    sub.setState(subState);
    const rootEnergy=Math.sqrt(n)*Math.sqrt(subConfig.density/2)*value;
    assert.equal(rootEnergy*rootEnergy,Number.MIN_VALUE);
    assert.equal(sub.getDiagnostics().kineticEnergy,rootEnergy*rootEnergy);
    assert.equal(sub.getDiagnostics().meanVx,value);sub.step(.01);
    assert.equal(sub.getDiagnostics().meanVx,value);assert.equal(sub.getDiagnostics().lastCellVisits,4*n);sub.delete();
    subState.vx.fill(0);for(const f of ['sigmaXX','sigmaYY','sigmaXY']) subState[f].fill(value);
    const subStress=new p.ElasticWaveGrid({...base,columns:512,rows:512,lambda:1e-308,shearModulus:1e-308});
    subStress.setState(subState);const sd=subStress.getDiagnostics();
    for(const f of ['meanSigmaXX','meanSigmaYY','meanSigmaXY']) assert.equal(sd[f],value);
    close(sd.meanSigmaZZ,.5*value,Number.MIN_VALUE);assert.equal(sd.strainEnergy,2*Number.MIN_VALUE);subStress.delete();
    console.log(`PASS: owned elastic plane-strain WASM; ${modes} independent P/S phase modes, six continuum refinements, adjoint/energy/means/compatibility, ownership/replay and range/late/work/clock rollback`);
}

function stress(p,probe) {
    assert.equal(typeof p.ElasticWaveGrid,'function');
    const temp=new p.ElasticWaveGrid(),base=temp.getConfig();temp.delete();
    const config={...base,columns:2,rows:2,maxSubstep:1};
    const g=new p.ElasticWaveGrid(config),budget=new p.ElasticWaveGrid({...config,maximumCellVisits:15});
    const late=new p.ElasticWaveGrid(config),large=zero(4);large.sigmaXX=[1.4e154,-1.4e154,1.4e154,-1.4e154];late.setState(large);
    const owners=[g,budget,late],states=owners.map(snapshot),ordinary=new Error('elastic foreign exception');
    const stack=()=>p._emscripten_stack_get_current();
    const nativeConfig=p.ElasticWaveGrid.prototype.getConfig;
    p.ElasticWaveGrid.prototype.getConfig=()=>({columns:1e20,rows:1e20});
    g.getConfig=()=>({columns:1e20,rows:1e20});
    function getter(throws) {const a=Array(4).fill(0);Object.defineProperty(a,3,{get(){throw throws;}});return a;}
    const fieldGetter={...zero(4),get sigmaXY(){throw ordinary;}};
    const proxy=new Proxy(Array(4).fill(0),{get(target,key){if(key==='3') throw ordinary;return Reflect.get(target,key);}});
    const lengthProxy=new Proxy(Array(4).fill(0),{get(target,key){if(key==='length') throw ordinary;return Reflect.get(target,key);}});
    const descriptorProxy=new Proxy(Array(4).fill(0),{getOwnPropertyDescriptor(target,key){if(key==='3') throw ordinary;return Reflect.getOwnPropertyDescriptor(target,key);}});
    const partialConfig={...config,get shearModulus(){throw ordinary;}};
    const failures=[
        [()=>budget.step(.01)], [()=>late.step(.35)],
        [()=>new p.ElasticWaveGrid(partialConfig),ordinary],
        [()=>new p.ElasticWaveGrid({...config,get maximumCellVisits(){throw ordinary;}}),ordinary],
        [()=>new p.ElasticWaveGrid({...config,maximumSubsteps:2**32+1})],
        [()=>g.setState({...zero(4),vx:getter(ordinary)}),ordinary],
        [()=>g.setState({...zero(4),vy:proxy}),ordinary],
        [()=>g.setState({...zero(4),sigmaXX:lengthProxy}),ordinary],
        [()=>g.setState({...zero(4),sigmaYY:descriptorProxy}),ordinary],
        [()=>g.setState(fieldGetter),ordinary],
        [()=>g.setState({...zero(4),sigmaXY:getter(73)}),73],
        [()=>g.setState({...zero(4),sigmaXY:getter(undefined)}),undefined],
        [()=>g.setState({...zero(4),sigmaXY:getter(null)}),null],
        [()=>g.setState({...zero(4),sigmaXY:getter('primitive')}),'primitive'],
        [()=>g.setState({...zero(4),sigmaXY:[0,0,0,Infinity]})],
        [()=>g.setState({...zero(4),sigmaXY:Array(4)})],
        [()=>g.setState({...zero(4),sigmaXY:[0,0,0,{valueOf(){throw ordinary;}}]})],
    ];
    function batch() {
        for(const item of failures) {
            const before=stack();
            assert.throws(item[0],error=>item.length===2?error===item[1]:error instanceof Error&&!('excPtr' in error));
            assert.equal(stack(),before);
        }
        // Every complete length, including the fifth, is checked before any entry.
        let reads=0;const unread=zero(4);Object.defineProperty(unread.vx,3,{get(){++reads;throw ordinary;}});unread.sigmaXY=[];
        assert.throws(()=>g.setState(unread),error=>error instanceof RangeError);assert.equal(reads,0);
        const nested=zero(4);Object.defineProperty(nested.sigmaXY,3,{get(){budget.step(.01);}});
        assert.throws(()=>g.setState(nested),error=>error instanceof Error&&!('excPtr' in error));
        const reentrant=zero(4);Object.defineProperty(reentrant.sigmaXY,3,{get(){
            g.setState(zero(4));assert.equal(probe.method(),7);return 0;
        }});g.setState(reentrant);
        const receiver=new p.ElasticWaveGrid(config),deleting=zero(4);
        Object.defineProperty(deleting.sigmaXY,3,{get(){receiver.delete();return 0;}});
        assert.throws(()=>receiver.setState(deleting),error=>error instanceof Error);assert.ok(receiver.isDeleted());
    }
    try {
        for(let i=0;i<20;++i) batch();
        const before=p.boundaryTestStats(),savedStack=stack();
        for(let i=0;i<1000;++i) batch();
        assert.deepEqual(p.boundaryTestStats(),before);assert.equal(stack(),savedStack);
        for(let i=0;i<owners.length;++i) assert.deepEqual(snapshot(owners[i]),states[i]);
        console.log(`PASS: elastic boundary stress; 1000 batches/20000 rejections, stack=${savedStack}, live heap=${before.heap}, uncaught=${before.uncaught}; unchanged five fields/clock/diagnostics`);
    } finally {
        p.ElasticWaveGrid.prototype.getConfig=nativeConfig;
        for(const owner of owners) owner.delete();
    }
}
module.exports={smoke,stress};
