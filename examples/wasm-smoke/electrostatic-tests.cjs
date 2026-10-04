const assert=require('node:assert/strict');
const config={columns:2,rows:2,spacingX:1,spacingY:1,permittivity:2};
const options={absoluteGaussTolerance:1e-10,relativeGaussTolerance:1e-10,maximumIterations:1000,maximumCellVisits:100000000};
const diagnostics='iterations cellVisits residualRestarts permittivity originalChargeMean effectiveChargeMean originalIntegratedCharge effectiveIntegratedCharge neutralityMeanAllowance removedChargeMean maximumSourceCorrection sourceCorrectionAllowance effectiveChargeRms targetGaussRms finalGaussRms maximumAbsGauss originalGaussRms maximumAbsOriginalGauss potentialMean meanFieldX meanFieldY curlRms maximumAbsCurl fieldEnergy sourceEnergy residualEnergyCorrection residualEnergyBound energyIdentityError roundoffEnergyAllowance hasSolution zeroSource'.split(' ');
const arrays=s=>[s.originalCharge,s.effectiveCharge,s.potential,s.field.xFaces,s.field.yFaces,s.gaussResidual,s.curl];
const close=(a,b,tol,label='')=>assert.ok(Math.abs(a-b)<=tol,`${label}: ${a} != ${b}; margin ${tol}`);
const rms=a=>Math.hypot(...a)/Math.sqrt(a.length);
const mean=a=>a.reduce((sum,x)=>sum+x,0)/a.length;
function mode(c,mx,my,amplitude=.7) {
    return Array.from({length:c.columns*c.rows},(_,k)=>amplitude*Math.cos(2*Math.PI*(mx*(k%c.columns+.5)/c.columns+my*(Math.floor(k/c.columns)+.5)/c.rows)+.31));
}
function eigenvalue(c,mx,my) {
    return 4*(Math.sin(Math.PI*mx/c.columns)/c.spacingX)**2+4*(Math.sin(Math.PI*my/c.rows)/c.spacingY)**2;
}
function charge(c,mx,my,amplitude=.7) {return mode(c,mx,my,amplitude).map(x=>x*c.permittivity*eigenvalue(c,mx,my));}
// Independently assembled reduced dense system, not an iterative stencil solve.
function dense(c,q) {
    const n=q.length,m=n-1,a=Array.from({length:m},()=>Array(m+1).fill(0));
    const at=(i,j)=>(i+c.columns)%c.columns+c.columns*((j+c.rows)%c.rows);
    for(let k=0;k<m;++k) {
        const i=k%c.columns,j=Math.floor(k/c.columns);
        for(const [neighbor,h] of [[at(i-1,j),c.spacingX],[at(i+1,j),c.spacingX],[at(i,j-1),c.spacingY],[at(i,j+1),c.spacingY]]) {
            const coefficient=c.permittivity/(h*h);a[k][k]+=coefficient;
            if(neighbor<m)a[k][neighbor]-=coefficient;
        }
        a[k][m]=q[k];
    }
    for(let k=0;k<m;++k) {
        let pivot=k;for(let i=k+1;i<m;++i)if(Math.abs(a[i][k])>Math.abs(a[pivot][k]))pivot=i;
        [a[k],a[pivot]]=[a[pivot],a[k]];assert.notEqual(a[k][k],0);
        for(let i=k+1;i<m;++i) {const ratio=a[i][k]/a[k][k];for(let j=k;j<=m;++j)a[i][j]-=ratio*a[k][j];}
    }
    const phi=Array(n).fill(0);
    for(let i=m-1;i>=0;--i) {let rhs=a[i][m];for(let j=i+1;j<m;++j)rhs-=a[i][j]*phi[j];phi[i]=rhs/a[i][i];}
    const gauge=mean(phi);return phi.map(x=>x-gauge);
}
function audit(s,c) {
    const n=c.columns*c.rows,V=c.spacingX*c.spacingY,r=[],original=[],curl=[];
    const at=(i,j)=>(i+c.columns)%c.columns+c.columns*((j+c.rows)%c.rows);
    let field=0,source=0,correction=0;
    for(let j=0;j<c.rows;++j)for(let i=0;i<c.columns;++i) {
        const k=at(i,j),left=at(i-1,j),down=at(i,j-1),right=at(i+1,j),up=at(i,j+1);
        close(s.field.xFaces[k],-(s.potential[k]-s.potential[left])/c.spacingX,2e-12);
        close(s.field.yFaces[k],-(s.potential[k]-s.potential[down])/c.spacingY,2e-12);
        const divergence=(s.field.xFaces[right]-s.field.xFaces[k])/c.spacingX+(s.field.yFaces[up]-s.field.yFaces[k])/c.spacingY;
        r[k]=c.permittivity*divergence-s.effectiveCharge[k];original[k]=c.permittivity*divergence-s.originalCharge[k];
        curl[k]=(s.field.yFaces[k]-s.field.yFaces[left])/c.spacingX-(s.field.xFaces[k]-s.field.xFaces[down])/c.spacingY;
        close(s.gaussResidual[k],r[k],2e-12);close(s.curl[k],curl[k],2e-12);
        field+=.5*c.permittivity*V*(s.field.xFaces[k]**2+s.field.yFaces[k]**2);
        source+=.5*V*s.potential[k]*s.effectiveCharge[k];correction+=.5*V*s.potential[k]*r[k];
    }
    const d=s.diagnostics;
    assert.deepEqual(Object.keys(d).sort(),diagnostics.slice().sort());
    for(const key of diagnostics)assert.equal(typeof d[key],key==='hasSolution'||key==='zeroSource'?'boolean':'number');
    for(const key of diagnostics)if(typeof d[key]==='number')assert.ok(Number.isFinite(d[key]),key);
    close(d.finalGaussRms,rms(r),2e-12);close(d.originalGaussRms,rms(original),2e-12);
    close(d.maximumAbsGauss,Math.max(...r.map(Math.abs)),2e-12);
    close(d.maximumAbsOriginalGauss,Math.max(...original.map(Math.abs)),2e-12);
    close(d.curlRms,rms(curl),2e-12);close(d.maximumAbsCurl,Math.max(...curl.map(Math.abs)),2e-12);
    close(d.fieldEnergy,field,2e-11);close(d.sourceEnergy,source,2e-11);
    close(d.residualEnergyCorrection,correction,2e-11);close(field-source,correction,3e-11);
    assert.ok(Math.abs(d.residualEnergyCorrection)<=d.residualEnergyBound+2e-12);
    assert.ok(Math.abs(d.energyIdentityError)<=d.roundoffEnergyAllowance);
    assert.ok(d.finalGaussRms<=d.targetGaussRms);
    close(d.potentialMean,mean(s.potential),2e-14);close(d.meanFieldX,mean(s.field.xFaces),2e-14);close(d.meanFieldY,mean(s.field.yFaces),2e-14);
    close(d.effectiveChargeRms,rms(s.effectiveCharge),2e-12);
    close(d.originalChargeMean,mean(s.originalCharge),2e-14);close(d.effectiveChargeMean,mean(s.effectiveCharge),2e-14);
    close(d.originalIntegratedCharge,mean(s.originalCharge)*n*V,2e-14);
    close(d.effectiveIntegratedCharge,mean(s.effectiveCharge)*n*V,2e-14);
    close(d.maximumSourceCorrection,Math.max(...s.originalCharge.map((x,k)=>Math.abs(x-s.effectiveCharge[k]))),0);
    assert.ok(d.maximumSourceCorrection<=d.sourceCorrectionAllowance);
}
function smoke(p) {
    assert.equal(typeof p.PeriodicElectrostaticGrid,'function');
    const defaults=new p.PeriodicElectrostaticGrid(),base=defaults.getConfig();
    assert.deepEqual(base,{columns:16,rows:16,spacingX:1,spacingY:1,permittivity:1});
    const initial=defaults.getSnapshot();for(const a of arrays(initial))assert.deepEqual(a,Array(256).fill(0));
    assert.equal(initial.diagnostics.hasSolution,false);base.columns=2;assert.equal(defaults.getConfig().columns,16);
    const zero=defaults.solve(Array(256).fill(0));assert.equal(zero.zeroSource,true);assert.equal(zero.iterations,0);assert.equal(zero.cellVisits,52*256);
    audit(defaults.getSnapshot(),defaults.getConfig());defaults.delete();
    const g=new p.PeriodicElectrostaticGrid(config),input=[1,-1,1,-1],d=g.solve(input),s=g.getSnapshot();
    assert.deepEqual(s.potential,[.125,-.125,.125,-.125]);assert.deepEqual(s.field.xFaces,[-.25,.25,-.25,.25]);
    assert.ok(s.field.yFaces.every(x=>x===0));close(d.fieldEnergy,.25,Number.EPSILON*.5);assert.equal(d.sourceEnergy,.25);
    assert.deepEqual(d,s.diagnostics);audit(s,config);
    const kept=g.getSnapshot();for(const a of arrays(s))a[0]=99;s.diagnostics.fieldEnergy=99;input[0]=99;
    assert.deepEqual(g.getSnapshot(),kept);
    const noIterations={...options,maximumIterations:0,absoluteGaussTolerance:2,relativeGaussTolerance:0};
    const loose=g.solve([1,-1,1,-1],noIterations);assert.equal(loose.iterations,0);assert.equal(loose.zeroSource,false);
    close(loose.finalGaussRms,1,Number.EPSILON);assert.equal(loose.fieldEnergy,0);assert.ok(loose.hasSolution);
    assert.ok(d.fieldEnergy-loose.fieldEnergy>.24); // Energy agreement of the zero field/source pair does not establish the analytic answer.
    g.solve([1,-1,1,-1]);const saved=g.getSnapshot();
    for(const fail of [()=>g.solve([1,1,1,1]),()=>g.solve([1,-1,1,-1],{...options,maximumIterations:0}),
        ()=>g.solve([1,-1,1,-1],{...options,maximumCellVisits:saved.diagnostics.cellVisits-1}),
        ()=>g.solve([1e200,-1e200,1e200,-1e200]),()=>g.solve([1e-200,-1e-200,1e-200,-1e-200],{...options,absoluteGaussTolerance:0})]) {
        assert.throws(fail);assert.deepEqual(g.getSnapshot(),saved);
    }
    g.delete();assert.equal(kept.potential[0],.125);
    let modes=0;
    for(const [nx,ny,mx,my] of [[2,2,1,0],[2,5,1,2],[5,2,2,1],[9,7,1,2]]) {
        const c={...config,columns:nx,rows:ny,spacingX:.23,spacingY:.41,permittivity:2.5};
        const grid=new p.PeriodicElectrostaticGrid(c),q=charge(c,mx,my),expected=mode(c,mx,my);
        grid.solve(q);const result=grid.getSnapshot();audit(result,c);
        for(let j=0;j<ny;++j)for(let i=0;i<nx;++i) {
            const k=i+nx*j;close(result.potential[k],expected[k],2e-12);
            close(result.field.xFaces[k],1.4*Math.sin(Math.PI*mx/nx)/c.spacingX*Math.sin(2*Math.PI*(mx*i/nx+my*(j+.5)/ny)+.31),2e-11);
            close(result.field.yFaces[k],1.4*Math.sin(Math.PI*my/ny)/c.spacingY*Math.sin(2*Math.PI*(mx*(i+.5)/nx+my*j/ny)+.31),2e-11);
        }
        for(const scale of [-1,1e-100,1e100]) {
            grid.solve(q.map(x=>x*scale),{...options,absoluteGaussTolerance:0,relativeGaussTolerance:1e-11});
            const scaled=grid.getSnapshot();for(let k=0;k<q.length;++k)close(scaled.potential[k]/scale,expected[k],2e-11);
            for(const key of ['xFaces','yFaces'])for(let k=0;k<q.length;++k)close(scaled.field[key][k]/scale,result.field[key][k],2e-10);
            close(scaled.diagnostics.fieldEnergy/(scale*scale),result.diagnostics.fieldEnergy,2e-10);
            assert.ok(scaled.diagnostics.finalGaussRms<=scaled.diagnostics.targetGaussRms);
        }
        grid.delete();++modes;
    }
    const c={...config,columns:2,rows:3,spacingX:.23,spacingY:.41,permittivity:2.5};
    const denseGrid=new p.PeriodicElectrostaticGrid(c),q=[1,-2,3,-1,2,-3];denseGrid.solve(q);
    const ds=denseGrid.getSnapshot(),exact=dense(c,ds.effectiveCharge);audit(ds,c);
    for(let k=0;k<q.length;++k)close(ds.potential[k],exact[k],3e-12);
    assert.throws(()=>denseGrid.solve(q,{...options,maximumIterations:1}));assert.deepEqual(denseGrid.getSnapshot(),ds);
    const replay=new p.PeriodicElectrostaticGrid(c);replay.solve(q);assert.deepEqual(replay.getSnapshot(),ds);replay.delete();denseGrid.delete();
    // Weighted Gauss remains finite even when raw div(E), or an opposing face
    // difference, overflows. Compare analytic normalized quantities, not that
    // deliberately unrepresentable unweighted intermediate.
    const extremeConfig={...config,spacingX:3e-154,spacingY:3e-154,permittivity:1e-308};
    const extreme=new p.PeriodicElectrostaticGrid(extremeConfig);
    for(const scale of [1,1e4]) {
        extreme.solve([1e150,-1e150,1e150,-1e150].map(x=>x*scale));const es=extreme.getSnapshot();
        for(let k=0;k<4;++k) {
            const sign=k%2?-1:1;
            close(es.potential[k]/(2.25e150*scale),sign,2e-14);
            close(es.field.xFaces[k]/(1.5e304*scale),-sign,2e-14);assert.ok(es.field.yFaces[k]===0);
        }
        close(es.diagnostics.fieldEnergy/(4.05e-7*scale*scale),1,2e-14);
        close(es.diagnostics.sourceEnergy/(4.05e-7*scale*scale),1,2e-14);
        assert.ok(es.diagnostics.finalGaussRms<=es.diagnostics.targetGaussRms);
        assert.ok(Math.abs(es.diagnostics.energyIdentityError)<=es.diagnostics.roundoffEnergyAllowance);
        assert.ok(es.diagnostics.curlRms===0);
    }
    extreme.delete();
    const neutral=new p.PeriodicElectrostaticGrid(config),near=[1,-1+1e-14,2,-2];neutral.solve(near);
    const ns=neutral.getSnapshot();assert.deepEqual(ns.originalCharge,near);audit(ns,config);
    assert.notEqual(ns.diagnostics.originalChargeMean,0);assert.ok(ns.diagnostics.maximumSourceCorrection>0);
    assert.ok(Math.abs(ns.diagnostics.originalChargeMean)<=ns.diagnostics.neutralityMeanAllowance);
    assert.throws(()=>neutral.solve([1,-1+1e-8,2,-2]));assert.deepEqual(neutral.getSnapshot(),ns);neutral.delete();
    let priorPhi=0,priorField=0;const refinements=[];
    for(const nx of [16,32,64,128]) {
        const cc={...config,columns:nx,rows:nx/2,spacingX:1/nx,spacingY:2/nx};
        const grid=new p.PeriodicElectrostaticGrid(cc),exact=mode(cc,1,1),q=exact.map(x=>x*cc.permittivity*8*Math.PI*Math.PI);
        grid.solve(q);const result=grid.getSnapshot();let ep=0,ef=0;
        for(let j=0;j<cc.rows;++j)for(let i=0;i<nx;++i) {
            const k=i+nx*j;ep+=(result.potential[k]-exact[k])**2;
            const ex=1.4*Math.PI*Math.sin(2*Math.PI*(i*cc.spacingX+(j+.5)*cc.spacingY)+.31);
            const ey=1.4*Math.PI*Math.sin(2*Math.PI*((i+.5)*cc.spacingX+j*cc.spacingY)+.31);
            ef+=(result.field.xFaces[k]-ex)**2+(result.field.yFaces[k]-ey)**2;
        }
        ep=Math.sqrt(ep/q.length);ef=Math.sqrt(ef/(2*q.length));
        if(priorPhi) {assert.ok(priorPhi/ep>3.9&&priorPhi/ep<4.1);assert.ok(priorField/ef>3.9&&priorField/ef<4.1);refinements.push([priorPhi/ep,priorField/ef]);}
        priorPhi=ep;priorField=ef;grid.delete();
    }
    for(const field of ['columns','rows'])for(const bad of [-1,0,1,.5,NaN,Infinity,262145,2**32,2**32+1,1e100])
        assert.throws(()=>new p.PeriodicElectrostaticGrid({...config,[field]:bad}));
    for(const field of ['spacingX','spacingY','permittivity'])for(const bad of [-1,0,NaN,Infinity])
        assert.throws(()=>new p.PeriodicElectrostaticGrid({...config,[field]:bad}));
    assert.throws(()=>new p.PeriodicElectrostaticGrid({...config,columns:512,rows:513}));
    assert.throws(()=>new p.PeriodicElectrostaticGrid({columns:2}));
    const validation=new p.PeriodicElectrostaticGrid(config);
    for(const field of Object.keys(options)) {
        const badCounts=field.startsWith('maximum')?[-1,.5,NaN,Infinity,2**32,2**32+1,1e100]:[-1,NaN,Infinity];
        for(const bad of badCounts)assert.throws(()=>validation.solve([1,-1,1,-1],{...options,[field]:bad}));
        const incomplete={...options};delete incomplete[field];assert.throws(()=>validation.solve([1,-1,1,-1],incomplete));
        for(const bad of [true,'1',{valueOf(){throw new Error('must not coerce options');}}])
            assert.throws(()=>validation.solve([1,-1,1,-1],{...options,[field]:bad}),e=>e instanceof TypeError);
    }
    assert.throws(()=>validation.solve([1,-1,1,-1],{...options,maximumIterations:1000001}));
    assert.throws(()=>validation.solve([1,-1,1,-1],{...options,maximumCellVisits:1000000001}));
    assert.throws(()=>validation.solve([0,0,0,0],{...options,maximumCellVisits:0}));
    assert.equal(validation.solve([0,0,0,0],{...options,maximumIterations:0,maximumCellVisits:52*4}).iterations,0);
    const zeroSaved=validation.getSnapshot();assert.throws(()=>validation.solve([0,0,0,0],{...options,maximumCellVisits:52*4-1}));
    assert.deepEqual(validation.getSnapshot(),zeroSaved);
    for(const bad of [[],[1,-1,1],Array(4),[1,-1,1,Infinity],new Float64Array(4),[1,-1,1,'-1']])assert.throws(()=>validation.solve(bad));
    validation.delete();
    console.log(`PASS: electrostatic WASM; ${modes} analytic/two-cell Fourier modes, dense gauge solve, continuum ratios ${JSON.stringify(refinements)}, full physical diagnostics, neutrality/energy/ownership/replay and late rollback`);
}
function stress(p,probe) {
    const g=new p.PeriodicElectrostaticGrid(config),source=[1,-1,1,-1];g.solve(source);const saved=g.getSnapshot();
    const error=new Error('electrostatic foreign failure'),stack=()=>p._emscripten_stack_get_current();
    const nativeConfig=p.PeriodicElectrostaticGrid.prototype.getConfig;
    p.PeriodicElectrostaticGrid.prototype.getConfig=()=>({columns:1e20,rows:1e20});g.getConfig=()=>({columns:1e20,rows:1e20});
    const getter=value=>{const a=source.slice();Object.defineProperty(a,3,{get(){throw value;}});return a;};
    const proxy=key=>new Proxy(source,{get(target,k){if(k===key)throw error;return Reflect.get(target,k);}});
    const descriptors=new Proxy(source,{getOwnPropertyDescriptor(target,k){if(k==='3')throw error;return Reflect.getOwnPropertyDescriptor(target,k);}});
    const failures=[
        [()=>g.solve([1,1,1,1])], [()=>g.solve(source,{...options,maximumIterations:0})],
        [()=>g.solve(source,{...options,maximumCellVisits:saved.diagnostics.cellVisits-1})],
        [()=>g.solve([1e200,-1e200,1e200,-1e200])], [()=>new p.PeriodicElectrostaticGrid({...config,get permittivity(){throw error;}}),error],
        [()=>g.solve(getter(error)),error], [()=>g.solve(proxy('3')),error], [()=>g.solve(proxy('length')),error],
        [()=>g.solve(descriptors),error], [()=>g.solve(getter(73)),73], [()=>g.solve(getter(undefined)),undefined],
        [()=>g.solve(getter(null)),null], [()=>g.solve(getter('primitive')),'primitive'],
        [()=>g.solve(Array(4))], [()=>g.solve([1,-1,1,Infinity])], [()=>g.solve(source,{...options,maximumIterations:2**32+1})],
    ];
    for(const key of Object.keys(options))failures.push([()=>g.solve(source,{...options,get [key](){throw error;}}),error]);
    failures.push([()=>g.solve(source,new Proxy(options,{get(target,key){if(key==='maximumCellVisits')throw error;return target[key];}})),error]);
    failures.push([()=>g.solve(source,{...options,get maximumCellVisits(){throw 73;}}),73]);
    const inherited=source.slice();delete inherited[3];Object.setPrototypeOf(inherited,{3:-1});failures.push([()=>g.solve(inherited)]);
    let total=0;
    function batch() {
        for(const item of failures) {
            const before=stack();assert.throws(item[0],e=>item.length===2?e===item[1]:e instanceof Error&&!('excPtr' in e));
            assert.equal(stack(),before);++total;
        }
        let reads=0;const wrong=source.slice(0,3);Object.defineProperty(wrong,0,{get(){++reads;throw error;}});
        const unreadOptions={get absoluteGaussTolerance(){++reads;throw error;}};
        assert.throws(()=>g.solve(wrong,unreadOptions),e=>e instanceof RangeError);assert.equal(reads,0);
        const nested=source.slice();Object.defineProperty(nested,3,{get(){g.solve(source,{...options,maximumIterations:0});}});
        assert.throws(()=>g.solve(nested),e=>e instanceof Error&&!('excPtr' in e));
        assert.throws(()=>g.solve(source,{...options,get maximumCellVisits(){g.solve(source,{...options,maximumIterations:0});}}));
        const reentrant=source.slice();Object.defineProperty(reentrant,3,{get(){assert.equal(probe.method(),7);return -1;}});g.solve(reentrant);
        g.solve(source,{...options,get maximumCellVisits(){assert.equal(probe.method(),7);return options.maximumCellVisits;}});
        const observed=[],readOptions={};
        for(const key of Object.keys(options))Object.defineProperty(readOptions,key,{get(){observed.push(key);return options[key];}});
        g.solve(source,readOptions);assert.deepEqual(observed,Object.keys(options));
        for(const optionsGetter of [false,true]) {
            const receiver=new p.PeriodicElectrostaticGrid(config),deleting=source.slice();
            const o=optionsGetter?{...options,get maximumCellVisits(){receiver.delete();return options.maximumCellVisits;}}:options;
            if(!optionsGetter)Object.defineProperty(deleting,3,{get(){receiver.delete();return -1;}});
            assert.throws(()=>receiver.solve(deleting,o),e=>e instanceof Error);assert.ok(receiver.isDeleted());
        }
    }
    try {
        for(let i=0;i<20;++i)batch();total=0;const before=p.boundaryTestStats(),savedStack=stack();
        for(let i=0;i<1000;++i)batch();assert.deepEqual(p.boundaryTestStats(),before);assert.equal(stack(),savedStack);
        assert.deepEqual(g.getSnapshot(),saved);
        console.log(`PASS: electrostatic boundary stress; 1000 batches/${total} rejection checks plus array/options reentrancy and deletion; stack=${savedStack}, live heap=${before.heap}, uncaught=${before.uncaught}`);
    }finally {p.PeriodicElectrostaticGrid.prototype.getConfig=nativeConfig;g.delete();}
}
module.exports={smoke,stress};
