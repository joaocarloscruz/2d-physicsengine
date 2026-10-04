const assert=require('node:assert/strict');
const fields=['ez','hx','hy'];
const zero=n=>Object.fromEntries(fields.map(f=>[f,Array(n).fill(0)]));
const snapshot=g=>({state:g.getState(),diagnostics:g.getDiagnostics()});
// Exact integer sum of represented float64 significands for the four-cell
// controls. This independent oracle uses neither scaled floating accumulation
// nor the evolution equations. The final test values fit this conversion range.
function mean4(values) {
    const view=new DataView(new ArrayBuffer(8));
    const samples=values.map(x=>{
        view.setFloat64(0,x);const bits=view.getBigUint64(0);
        const stored=Number((bits>>52n)&2047n),sign=bits>>63n?-1n:1n;
        return {mantissa:sign*((bits&((1n<<52n)-1n))+(stored?1n<<52n:0n)),exponent:stored?stored-1075:-1074};
    }).filter(x=>x.mantissa!==0n);
    if(samples.length===0)return 0;
    const exponent=Math.min(...samples.map(x=>x.exponent));
    const sum=samples.reduce((a,x)=>a+(x.mantissa<<BigInt(x.exponent-exponent)),0n);
    return Number(sum)*2**exponent/values.length;
}
function coherent(g) {
    const s=g.getState(),d=g.getDiagnostics();
    for(const f of fields)assert.equal(d['mean'+f[0].toUpperCase()+f.slice(1)],mean4(s[f]),f);
}
function smoke(p) {
    const def=new p.MaxwellGrid(),base=def.getConfig();def.delete();
    for(const x of [.1,.7,-1.2345678901234567]) {
        const g=new p.MaxwellGrid({...base,columns:3,rows:2});
        g.setState({ez:Array(6).fill(x),hx:Array(6).fill(-x),hy:Array(6).fill(x)});
        function check() {
            const s=g.getState(),d=g.getDiagnostics();
            assert.equal(d.meanEz,s.ez[0]);assert.equal(d.meanHx,s.hx[0]);assert.equal(d.meanHy,s.hy[0]);
        }
        check();g.step(.01);check();g.stepOhmic(.01,.7);check();g.delete();
    }
    const c={...base,columns:512,rows:512,permittivity:1e308,permeability:1e308},n=512*512;
    for(const magnitude of [6e-319,7e-319])for(const sign of [-1,1]) {
        const g=new p.MaxwellGrid(c),x=sign*magnitude,s={ez:Array(n).fill(x),hx:Array(n).fill(-x),hy:Array(n).fill(x)};
        const weighted=Math.sqrt(n)*Math.sqrt(c.permittivity/2)*magnitude;
        const electric=weighted*weighted,magnetic=(Math.sqrt(2)*weighted)**2;
        assert.equal(electric,Number.MIN_VALUE);assert.ok(magnetic>0);
        g.setState(s);
        function check() {
            const d=g.getDiagnostics();assert.equal(d.meanEz,x);assert.equal(d.meanHx,-x);assert.equal(d.meanHy,x);
            assert.equal(d.electricEnergy,electric);assert.equal(d.magneticEnergy,magnetic);
            assert.deepEqual(g.getState(),s);
        }
        check();assert.equal(g.getModifiedEnergy(.01),electric+magnetic);
        g.step(.01);check();assert.equal(g.getDiagnostics().lastCellVisits,4*n);
        assert.equal(g.stepOhmic(.01,0).cellVisits,4*n);check();
        const copy=g.getState();copy.ez[0]=0;assert.equal(g.getDiagnostics().meanEz,x);g.delete();
    }
    const small={...base,columns:2,rows:2},g=new p.MaxwellGrid(small);
    // All permutations retain the same small represented component.
    function permutations(v) {
        if(v.length===0)return [[]];
        return v.flatMap((x,k)=>permutations(v.filter((_,i)=>i!==k)).map(t=>[x,...t]));
    }
    for(const values of permutations([1e16,1,-1e16,0]))for(const sign of [-1,1]) {
        const s={ez:values.map(x=>sign*x),hx:values.map(x=>-sign*x),hy:values.map(x=>sign*x)};
        g.setState(s);assert.equal(g.getDiagnostics().meanEz,sign*.25);
        assert.equal(g.getDiagnostics().meanHx,-sign*.25);assert.equal(g.getDiagnostics().meanHy,sign*.25);
    }
    g.setState({ez:[1e16,1,-1e16,0],hx:[1e16,1,-1e16,0],hy:[-1e16,-1,1e16,0]});
    g.step(.01);coherent(g);assert.equal(g.getDiagnostics().lastCellVisits,16);
    assert.equal(g.stepOhmic(.01,.7).cellVisits,36);coherent(g);g.delete();
    const sub=new p.MaxwellGrid({...c,permeability:1}),s=zero(n);s.ez.fill(6e-319);sub.setState(s);
    const before=snapshot(sub);
    for(let k=0;k<n;++k)s.ez[k]=k%2?-6e-319:6e-319;
    s.ez[1]+=Number.MIN_VALUE;
    assert.throws(()=>sub.setState(s),e=>e instanceof Error&&/mean underflows/.test(e.message));
    assert.deepEqual(snapshot(sub),before);sub.delete();
    console.log('PASS: Maxwell binary-scaled means; signed subnormal constants/aggregate energy, all 48 cancellation orders/signs, exact stored-float64 means after both stepping paths, final-underflow rollback');
}
function stress(p) {
    const def=new p.MaxwellGrid(),base=def.getConfig();def.delete();
    const g=new p.MaxwellGrid({...base,columns:2,rows:2}),s=zero(4);s.ez.fill(1);g.setState(s);g.stepOhmic(.01,.7);
    const before=snapshot(g),bad=[];
    const partial=3*Number.MIN_VALUE*2**497;
    for(const values of [[1e150,Number.MIN_VALUE,-1e150,0],[Number.MIN_VALUE,1e150,-1e150,0],
        [1e150,partial,-1e150,0],[partial,1e150,-1e150,0]])for(const f of fields) {
        const a=zero(4);a[f]=values.slice();bad.push(a);
    }
    const stack=()=>p._emscripten_stack_get_current();
    function batch() {
        for(const a of bad) {
            const s=stack();assert.throws(()=>g.setState(a),e=>e instanceof Error&&/mean scaling/.test(e.message)&&!('excPtr' in e));
            assert.equal(stack(),s);
        }
    }
    for(let k=0;k<20;++k)batch();const stats=p.boundaryTestStats(),s0=stack();
    for(let k=0;k<1000;++k)batch();
    assert.deepEqual(p.boundaryTestStats(),stats);assert.equal(stack(),s0);assert.deepEqual(snapshot(g),before);g.delete();
    console.log(`PASS: Maxwell mean boundary stress; 1000 batches/12000 observer rejections, stack=${s0}, live heap=${stats.heap}, uncaught=${stats.uncaught}; complete rollback`);
}
module.exports={smoke,stress};
