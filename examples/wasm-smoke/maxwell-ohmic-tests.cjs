const assert = require('node:assert/strict');
const fields = ['ez','hx','hy'];
const snapshot = g => ({state:g.getState(), diagnostics:g.getDiagnostics()});
const zero = n => Object.fromEntries(fields.map(f => [f,Array(n).fill(0)]));
function close(a,b,tol,label='') {
    assert.ok(Math.abs(a-b)<=tol, `${label}: ${a} != ${b}; margin ${tol}`);
}
function compare(a,b,tol) {
    for(const f of fields) for(let k=0;k<a[f].length;++k) close(a[f][k],b[f][k],tol,`${f}[${k}]`);
}
function difference(a,b) {
    let error=0;
    for(const f of fields) for(let k=0;k<a[f].length;++k) error=Math.max(error,Math.abs(a[f][k]-b[f][k]));
    return error;
}
function mode(c,mx,my,e,b,continuum=false) {
    const x=continuum?2*Math.PI*mx/(c.columns*c.spacingX):2*Math.sin(Math.PI*mx/c.columns)/c.spacingX;
    const y=continuum?2*Math.PI*my/(c.rows*c.spacingY):2*Math.sin(Math.PI*my/c.rows)/c.spacingY;
    const s=zero(c.columns*c.rows);
    for(let j=0;j<c.rows;++j) for(let i=0;i<c.columns;++i) {
        const k=i+c.columns*j,p=2*Math.PI*(i*mx/c.columns+j*my/c.rows)+.31;
        s.ez[k]=e*Math.cos(p);
        s.hx[k]=y*b*Math.sin(p+Math.PI*my/c.rows);
        s.hy[k]=-x*b*Math.sin(p+Math.PI*mx/c.columns);
    }
    return s;
}
// Closed damped oscillator from the unsplit equations, not a copy of split stepping.
function exact(omega,a,mu,t,regime) {
    const half=a/2,d=Math.exp(-half*t);
    if(regime===1) return [d*(1-half*t),d*t/mu];
    const frequency=Math.sqrt(Math.abs(omega*omega-half*half));
    const sinc=(regime===0?Math.sin(frequency*t):Math.sinh(frequency*t))/frequency;
    const cosine=regime===0?Math.cos(frequency*t):Math.cosh(frequency*t);
    return [d*(cosine-half*sinc),d*sinc/mu];
}
// Independent physical quadrature and spatial gradient, retaining anisotropic incidence.
function energy(c,s,h=0) {
    let electric=0,magnetic=0,correction=0;
    const V=c.spacingX*c.spacingY;
    for(let j=0;j<c.rows;++j) for(let i=0;i<c.columns;++i) {
        const k=i+c.columns*j;
        electric+=V*c.permittivity/2*s.ez[k]**2;
        magnetic+=V*c.permeability/2*(s.hx[k]**2+s.hy[k]**2);
        const x=(s.ez[(i+1)%c.columns+c.columns*j]-s.ez[k])/c.spacingX;
        const y=(s.ez[i+c.columns*((j+1)%c.rows)]-s.ez[k])/c.spacingY;
        correction+=V*h*h/(8*c.permeability)*(x*x+y*y);
    }
    return {electric,magnetic,physical:electric+magnetic,modified:electric+magnetic-correction,correction};
}
function roundoff(scale,n,steps=1) {
    const count=128*n*(steps+1),u=Number.EPSILON;
    return scale*count*u/(1-count*u); // Actual energy scale; no one-unit floor.
}
function smoke(p) {
    const defaults=new p.MaxwellGrid(),base=defaults.getConfig();defaults.delete();
    const dcConfig={...base,columns:3,rows:2,spacingX:.2,spacingY:.4,permittivity:2,permeability:3};
    const dc=new p.MaxwellGrid(dcConfig),dcState={ez:Array(6).fill(2),hx:Array(6).fill(3),hy:Array(6).fill(-4)};
    dc.setState(dcState);
    const report=dc.stepOhmic(.3,1.7),U=.2*.4*2*.5*6*4;
    compare(dc.getState(),{...dcState,ez:Array(6).fill(2*Math.exp(-1.7*.3/2))},8e-15);
    close(report.exactJouleEnergy,U*(-Math.expm1(-2*1.7*.3/2)),roundoff(U,6,report.substeps));
    assert.equal(report.modifiedEnergyDissipation,report.exactJouleEnergy);
    assert.equal(report.wavePhysicalEnergyChange,0);assert.equal(report.cellVisits,6*(8*report.substeps+1));
    assert.deepEqual(Object.keys(report).sort(),['conductivity','duration','startTime','endTime',
        'initialPhysicalEnergy','finalPhysicalEnergy','exactJouleEnergy','representedElectricEnergyLoss',
        'wavePhysicalEnergyChange','modifiedEnergyDissipation','decayStorageEnergyChange',
        'physicalBalanceResidual','substep','substeps','cellVisits'].sort());
    const saved=snapshot(dc);report.finalPhysicalEnergy=123;report.endTime=100;
    assert.deepEqual(snapshot(dc),saved);dc.delete();
    let temporal=0;
    for(const [nx,ny] of [[12,10],[2,5],[5,2],[2,2]]) for(const regime of [0,1,2]) {
        const c={...base,columns:nx,rows:ny,spacingX:.23,spacingY:.41,permittivity:1.7,permeability:.8,maxSubstep:1};
        const mx=nx===2?1:2,my=ny===2?1:2,x=2*Math.sin(Math.PI*mx/nx)/c.spacingX,y=2*Math.sin(Math.PI*my/ny)/c.spacingY;
        const omega=Math.hypot(x,y)/Math.sqrt(c.permittivity*c.permeability),a=[.6,2,3][regime]*omega,T=.17;
        const initial=mode(c,mx,my,1,0),target=mode(c,mx,my,...exact(omega,a,c.permeability,T,regime));
        const exactHeat=energy(c,initial).physical-energy(c,target).physical,errors=[],heatErrors=[];
        for(const steps of [20,40,80]) {
            const g=new p.MaxwellGrid(c);g.setState(initial);let heat=0;
            for(let k=0;k<steps;++k) heat+=g.stepOhmic(T/steps,a*c.permittivity).exactJouleEnergy;
            errors.push(difference(g.getState(),target));heatErrors.push(Math.abs(heat-exactHeat));
            assert.ok(g.getDiagnostics().maxAbsMagneticDivergence<3e-13);g.delete();
        }
        for(let k=1;k<3;++k) for(const e of [errors,heatErrors])
            assert.ok(e[k]/e[k-1]>.18&&e[k]/e[k-1]<.31,`temporal ratio ${e[k]/e[k-1]}`);
        ++temporal;
    }
    for(const [mx,my] of [[1,0],[0,1],[1,2]]) {
        const errors=[],T=.3;
        for(const n of [16,32,64]) {
            const c={...base,columns:n,rows:n,spacingX:2/n,spacingY:3/n,permittivity:1.7,permeability:.8,maxSubstep:1};
            const omega=Math.hypot(Math.PI*mx,2*Math.PI*my/3)/Math.sqrt(c.permittivity*c.permeability);
            const g=new p.MaxwellGrid(c);g.setState(mode(c,mx,my,1,0,true));
            const steps=Math.ceil(T/(.2*g.getStableTimeStep()));
            for(let k=0;k<steps;++k)g.stepOhmic(T/steps,1.3);
            errors.push(difference(g.getState(),mode(c,mx,my,...exact(omega,1.3/c.permittivity,c.permeability,T,0),true)));
            g.delete();
        }
        for(let k=1;k<3;++k)assert.ok(errors[k]/errors[k-1]>.18&&errors[k]/errors[k-1]<.31);
    }
    // Independent single-mode matrix for one wave stage between scalar decays.
    const c={...base,columns:9,rows:7,spacingX:.2,spacingY:.35,permittivity:2,permeability:3,maxSubstep:1};
    const g=new p.MaxwellGrid(c),h=.7*g.getStableTimeStep(),sigma=.8;
    const initial=mode(c,2,1,1,.03),r=Math.exp(-sigma*h/(2*c.permittivity)),f=-Math.expm1(-sigma*h/c.permittivity);
    const k2=(2*Math.sin(2*Math.PI/9)/c.spacingX)**2+(2*Math.sin(Math.PI/7)/c.spacingY)**2;
    const e1=r,bhalf=.03+h*e1/(2*c.permeability),ew=e1-h*k2*bhalf/c.permittivity,bw=bhalf+h*ew/(2*c.permeability);
    const before=energy(c,initial,h),d1=energy(c,mode(c,2,1,e1,.03),h),w=energy(c,mode(c,2,1,ew,bw),h),d2=energy(c,mode(c,2,1,r*ew,bw),h);
    g.setState(initial);
    const ledger=g.stepOhmic(h,sigma),budget=roundoff(before.physical,63);
    compare(g.getState(),mode(c,2,1,r*ew,bw),2e-15);
    close(ledger.exactJouleEnergy,f*(before.electric+w.electric),budget);
    close(ledger.modifiedEnergyDissipation,f*(before.electric-before.correction+w.electric-w.correction),budget);
    close(ledger.wavePhysicalEnergyChange,w.physical-d1.physical,budget);
    close(ledger.representedElectricEnergyLoss,before.electric-d1.electric+w.electric-d2.electric,budget);
    assert.ok(ledger.exactJouleEnergy>ledger.modifiedEnergyDissipation);
    assert.ok(Math.abs(ledger.wavePhysicalEnergyChange)>1e-6);
    let previous=g.getModifiedEnergy(h);
    const upper=before.modified/(1-(h*g.getWaveSpeed()*Math.hypot(1/c.spacingX,1/c.spacingY))**2);
    const divergence=g.getMagneticDivergence();
    for(let k=0;k<200;++k) {
        const l=g.stepOhmic(h,sigma),next=g.getModifiedEnergy(h),tol=roundoff(before.physical,63,k+2);
        assert.ok(next<=previous+tol);close(previous-next,l.modifiedEnergyDissipation,tol);
        assert.ok(g.getDiagnostics().totalEnergy<=upper+tol);
        assert.ok(Math.abs(l.physicalBalanceResidual)<=tol);previous=next;
    }
    g.getMagneticDivergence().forEach((v,k)=>close(v,divergence[k],3e-13));g.delete();
    const means=new p.MaxwellGrid(c),mixed=mode(c,2,1,.7,.1);
    for(let j=0;j<c.rows;++j)for(let i=0;i<c.columns;++i) {
        const k=i+c.columns*j;mixed.ez[k]+=.4;mixed.hx[k]+=1.2+.2*Math.sin(2*Math.PI*i/c.columns);mixed.hy[k]-=.3;
    }
    means.setState(mixed);const div0=means.getMagneticDivergence();assert.ok(Math.max(...div0.map(Math.abs))>.1);
    const meanLedger=means.stepOhmic(.3,sigma),meansD=means.getDiagnostics();
    close(meansD.meanEz,.4*Math.exp(-sigma*.3/c.permittivity),2e-14);
    close(meansD.meanHx,1.2,2e-14);close(meansD.meanHy,-.3,2e-14);
    means.getMagneticDivergence().forEach((v,k)=>close(v,div0[k],3e-13));
    assert.ok(Math.abs(meanLedger.physicalBalanceResidual)<=roundoff(meanLedger.initialPhysicalEnergy,63,meanLedger.substeps));means.delete();
    const small={...base,columns:2,rows:2},old=new p.MaxwellGrid(small),ohm=new p.MaxwellGrid(small);
    old.setState(mode(small,1,1,.7,.1));ohm.setState(old.getState());
    for(const dt of [.01,0,.04,.08]) {
        old.step(dt);const l=ohm.stepOhmic(dt,0);
        assert.deepEqual(snapshot(ohm),snapshot(old));assert.equal(l.exactJouleEnergy,0);assert.equal(l.modifiedEnergyDissipation,0);
        assert.equal(l.cellVisits,dt===0?0:old.getDiagnostics().lastCellVisits);
    }
    const noop=snapshot(ohm);assert.equal(ohm.stepOhmic(0,3).cellVisits,0);assert.deepEqual(snapshot(ohm),noop);
    old.delete();ohm.delete();
    const tight=new p.MaxwellGrid({...small,maximumCellVisits:16}),tightState=zero(4);tightState.ez.fill(1);tight.setState(tightState);
    assert.equal(tight.stepOhmic(.1,0).cellVisits,16);
    const tightBefore=snapshot(tight);assert.throws(()=>tight.stepOhmic(.1,1));assert.deepEqual(snapshot(tight),tightBefore);tight.delete();
    const tiny=new p.MaxwellGrid(small),s=zero(4);s.ez.fill(1);tiny.setState(s);
    const tinyLedger=tiny.stepOhmic(.1,1e-18);assert.deepEqual(tiny.getState(),s);
    assert.ok(tinyLedger.exactJouleEnergy>0);close(tinyLedger.exactJouleEnergy,4e-19,4e-34);
    assert.equal(tinyLedger.representedElectricEnergyLoss,0);
    assert.equal(tinyLedger.decayStorageEnergyChange,tinyLedger.exactJouleEnergy);tiny.delete();
    const strong=new p.MaxwellGrid(small);strong.setState(s);strong.stepOhmic(.1,1000);
    strong.getState().ez.forEach(e=>{assert.ok(e>0);close(e,Math.exp(-100),4e-15*Math.exp(-100));});strong.delete();
    const scaled=new p.MaxwellGrid({...small,permittivity:1e-308});s.ez.fill(1e154);scaled.setState(s);
    scaled.stepOhmic(1e-310,100);assert.equal(100/1e-308,Infinity);
    const expected=1e154*Math.exp(-(100*1e-310)/1e-308);
    scaled.getState().ez.forEach(e=>close(e,expected,3e-15*expected));scaled.delete();
    const a=new p.MaxwellGrid(small),b=new p.MaxwellGrid(small);a.setState(mode(small,1,1,.7,.1));b.setState(a.getState());
    const retained=[];
    for(const dt of [.021,0,.137,.004]) {
        const l=a.stepOhmic(dt,.7),copy=structuredClone(l);retained.push([l,copy]);
        assert.deepEqual(l,b.stepOhmic(dt,.7));assert.deepEqual(snapshot(a),snapshot(b));
    }
    a.delete();b.delete();for(const [l,copy] of retained)assert.deepEqual(l,copy);
    console.log(`PASS: Ohmic Maxwell DC, ${temporal} damped-mode/heat temporal sequences, 3 continuum refinements, split stage oracle, fixed-h Q, zero sigma, range and owning replay`);
}
function stress(p,probe) {
    const def=new p.MaxwellGrid(),base=def.getConfig();def.delete();
    const c={...base,columns:2,rows:2,maxSubstep:1},normal=new p.MaxwellGrid(c);
    const budget=new p.MaxwellGrid({...c,maximumCellVisits:35});
    const late=new p.MaxwellGrid(c);late.setState(mode(c,1,1,0,3e153));
    const clock=new p.MaxwellGrid({...c,permittivity:1e40,permeability:1e40,maxSubstep:1e14});clock.step(1e14);
    const tiny=new p.MaxwellGrid({...c,permittivity:1e200});const tinyState=zero(4);tinyState.ez.fill(1e-100);tiny.setState(tinyState);
    const owners=[normal,budget,late,clock,tiny],before=owners.map(snapshot),ordinary=new Error('original Ohmic conversion failure');
    const badScalar={valueOf(){throw ordinary;}};
    const failures=[()=>normal.stepOhmic(0,-1),()=>normal.stepOhmic(-1,1),()=>normal.stepOhmic(0,NaN),
        ()=>normal.stepOhmic(0,Infinity),()=>normal.stepOhmic(Infinity,0),()=>normal.stepOhmic(.1,Number.MIN_VALUE),
        ()=>normal.stepOhmic(.1,1e7),()=>budget.stepOhmic(.01,1),()=>late.stepOhmic(.6,.1),
        ()=>clock.stepOhmic(1e-10,1),()=>tiny.stepOhmic(.1,1.2e204),()=>normal.stepOhmic(.1,badScalar),
        ()=>normal.stepOhmic(badScalar,.1)];
    const stack=()=>p._emscripten_stack_get_current();
    function batch() {
        for(const fail of failures) {
            const s=stack();assert.throws(fail,e=>e===ordinary||(e instanceof Error&&!('excPtr' in e)));
            assert.equal(stack(),s);
        }
        // Release SDK converters coerce doubles; assertion-enabled converters
        // reject nonnumbers first. If coercion occurs, preserve original identity.
        let conversions=0;
        const nested={valueOf(){++conversions;assert.equal(probe.method(),7);throw ordinary;}};
        assert.throws(()=>normal.stepOhmic(.1,nested),e=>conversions?e===ordinary:e instanceof TypeError);
        assert.deepEqual(normal.stepOhmic(0,.5),normal.stepOhmic(0,.5));
    }
    for(let k=0;k<20;++k)batch();
    const stats=p.boundaryTestStats(),s=stack();
    for(let k=0;k<1000;++k)batch();
    assert.deepEqual(p.boundaryTestStats(),stats);assert.equal(stack(),s);
    owners.forEach((g,k)=>{assert.deepEqual(snapshot(g),before[k]);g.delete();});
    console.log(`PASS: Ohmic Maxwell boundary stress; 1000 batches/14000 rejection checks, stack=${s}, live heap=${stats.heap}, uncaught=${stats.uncaught}; complete rollback`);
}
module.exports={smoke,stress};
