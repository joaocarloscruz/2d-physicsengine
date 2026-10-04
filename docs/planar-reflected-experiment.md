# Source-owned planar reflection experiment (#77)

This standalone experiment establishes a conservative pressure/continuity map
for a single explicit moving plane. It removes the independent wall sampling
phase in a regular lattice, but **does not solve arbitrary particle disorder or
row phase consistency**. It is not integrated into WCSPH, DFSPH, or rigid-body
coupling. All nine original #44 expected failures and their thresholds remain.
There is no time integration, stability claim, mass calibration, density reset,
kernel normalization change, hydrostatic extrapolation, viscosity, or gravity.

The implementation is [planar_reflected_operator.h](../benchmarks/planar_reflected_operator.h),
with [analytical tests](../tests/test_planar_reflected_operator.cpp) and a
[diagnostic executable](../benchmarks/planar_reflected_diagnostic.cpp). Its local
double-vector/state types are deliberately outside the production API. There
are at most 1024 sources, one plane, one fixed support radius h, and O(N²) work.
Invalid geometry/state and non-finite results throw. Sources must be strictly
inside the fluid half-plane; corners, intersections, curves and sources on the
plane are unsupported.

## Geometry and ownership

Let the unit normal n point into the fluid, a be a point on the plane, and C
be the origin used to specify its rigid motion. J rotates a vector by +90°.
The wall velocity is v_b(q) = V + ω J(q-C). For source j,

```
d_j = n·(x_j-a) > 0
q_j = x_j-d_j n
R   = I-2nnᵀ
x_gj = x_j-2d_j n
b_j = 2nnᵀ J(q_j-C) - 2d_j Jn
v_gj = R v_j + 2nnᵀ V + ω b_j
     = R v_j + 2nnᵀ v_b(q_j) - 2d_j ω Jn.
```

Here v_gj is the derivative of the actual reflected position as the source
moves and the plane translates/rotates. The last term is required: reflecting
only the relative normal velocity at q_j fails joint rigid rotation for an
off-diagonal source/receiver pair. Wall tangent translation leaves the infinite
plane geometry unchanged, so no tangential wall force is generated. This is
slip kinematics, with no tangential drag or no-slip enforcement.

Each reflected quadrature point retains source j's mass and h. Its thermodynamic
ownership is also j; it is not a separate fluid degree of freedom, mass, density,
pressure, or internal-energy reservoir. Owner density ρ_j and pressure p_j are
the explicitly supplied prepared state. They are **not** a separately
extrapolated ghost pressure. The tool records density quadrature separately:

```
σ_i = Σ_j m_j [W(x_i-x_j) + W(x_i-x_gj)].
```

It never silently replaces ρ_i with σ_i. The physical two-dimensional kernel
normalizations remain unchanged. Mass is the nominal m = ρ dx² in the lattice
experiments. Pressure and density use the corresponding simulation units;
dimensionally ρ is mass/area and p is mass/time² in a two-dimensional model.

## Continuity and its complete pressure adjoint

For G_ij = ∇W_pressure(x_i-x_gj), the directed ghost row contributes

```
ρdot_i += m_j (v_i-v_gj)·G_ij.
λ_i = m_i p_i / ρ_i².
```

The pressure forces are the negative transpose of this same velocity map,
weighted by λ. Each directed interaction accumulates

```
f_i     += -λ_i m_j G_ij
f_j     += +λ_i m_j R G_ij
F_wall  += +2 λ_i m_j n(n·G_ij)
τ_wall,C += λ_i m_j b_j·G_ij.
```

Both fluid contributions accumulate when i=j. The real-fluid directed rows
use m_j(v_i-v_j)·∇W_pressure(x_i-x_j), with their corresponding opposite forces.
All source and receiver rows are included once. The usual symmetric pair
pressure appears after combining reciprocal directed rows.

τ_wall,C is the **complete generalized torque about C**. Applying F_wall at
q_j accounts for only part of it; the remaining interaction couple is

```
τ_couple = -2 λ_i m_j d_j (Jn·G_ij).
```

This couple cannot be discarded for unequal owner pressure duals. The tool
reports projection torque and the remaining couple separately. It computes an
explicit experimental plane wrench; it does not deliver that wrench through
the production rigid-body coupler. Changing C requires the corresponding
change to V and torque; wall work remains V·F_wall + ω τ_wall,C.

For radial gradients, this construction gives, to floating-point error,

```
Σ_i f_i + F_wall = 0
Σ_i x_i×f_i + C×F_wall + τ_wall,C = 0
Σ_i v_i·f_i + V·F_wall + ω τ_wall,C + Σ_i λ_i ρdot_i = 0.
```

The last term is the owner internal-energy rate if du_i/dρ_i = p_i/ρ_i².
For a fixed wall its work is zero; a prescribed moving wall supplies/removes
the explicitly reported mechanical work. No separately extrapolated pressure
reservoir appears in this construction. Adding one later requires an explicit
energy-exchange model and a new adjoint derivation.

For the **matched cubic** family, G is the derivative of W, so this continuity
is also dσ_i/dt for fixed mass/h. Tests differentiate the reflected geometry,
density quadrature, and a frozen-dual potential independently by central finite
differences. For **legacy poly6/spiky**, G is the spiky pressure gradient while
density uses poly6. Its formal pressure/continuity work identity still holds,
but the continuity map is not the derivative of σ. A test explicitly preserves
this mismatch rather than claiming variational density consistency.

## Reproduction and numerical evidence

Build with BUILD_TESTING enabled and run:

```
cmake --build build --target planar_reflected_diagnostic engine_tests_runner --parallel 2
build/planar_reflected_diagnostic --output planar-reflected.json
build/run_tests "[planar-reflection]"
```

On Windows use the `.exe` suffix. `--quick` runs eight configurations for CTest;
the full sweep runs 72. Unknown flags and inaccessible output paths exit 1.
[Committed results](data/planar-reflected-582cf005-clang23.json) identify the
source base and operator commit, using Clang 23.1.1 / Release / Windows x86_64.
Tests pass 106 assertions in 10 new cases; all ten native CTest entries pass.
All nine original #44 tests still fail as expected with unchanged thresholds.

Canonical static configurations use ρ=1000, p=1000, m=1000 dx², plane y=0,
and source rows y=(j+1/2)dx. Width/depth exceed the complete support of the
reported central first-row and central bulk samples. Finite patch outer edges
are **not** pressure-free surfaces or used as a local equilibrium acceptance
test; global force/work identities nevertheless include every finite source.
Uniform tangent velocity is (.4,0). Joint rigid motion uses C=(.12,-.17),
V=(.2,-.3), ω=.7. Compression uses v=-.1 x, with continuum ρdot/ρ=.2.
These are instantaneous measurements, with no duration or timestep.

Controls retain reflected source ownership in every case:

* `aligned`: identical tangential phase in all rows.
* `common_phase`: every source shifted tangentially by .37 dx.
* `row_phase`: alternating rows shifted tangentially by .5 dx.
* `disorder`: x perturbation .12 dx sin(1.7i+2.3j+.4), y perturbation
  .12 dx cos(2.1i-1.3j+.7), deterministic in lattice indices.

Common phase is a symmetry of the plane, unlike row phase or disorder. At
dx=.1 and h/dx=2.5:

| Family/control | σ_bottom/ρ | f_bottom,y/(p dx) | Bottom acceleration magnitude | Compression bottom ρdot/ρ |
| --- | ---: | ---: | ---: | ---: |
| Legacy aligned / common phase | .993975943 | <3e-16 | <3e-15 | .184961562 |
| Cubic aligned / common phase | .999448487 | <3e-16 | <3e-15 | .200209237 |
| Legacy row phase | .996114170 | +.004348862 | .043488616 | .186020310 |
| Cubic row phase | 1.000771586 | -.023055152 | .230551523 | .202472352 |
| Legacy disorder | .964192419 | -.233026336 | 2.599919559 | .185235360 |
| Cubic disorder | .942843931 | -.222997340 | 2.494069048 | .206972648 |

Disorder also produces a tangential force: f_x/(p dx)=-.115301969 legacy,
-.111695974 cubic. Cubic disorder bulk acceleration is 1.943359654, so this is
not exclusively a wall error. At fixed h/dx=2.5, dx=.1/.05/.025 leaves the
dimensionless density/force/compression defects unchanged, while cubic bottom
acceleration grows 2.494069/4.988138/9.976276. **Spatial refinement at fixed
neighbor ratio does not remove this scale-invariant quadrature defect.**

Increasing neighbors gives encouraging but bounded evidence. Cubic disorder
at fixed h=.2, dx=.1/.05/.025 (h/dx=2/4/8) gives bottom acceleration
4.409308/.461075/.023264, bulk acceleration .253578/.050014/.024734, and
bottom σ/ρ=.910327/.994271/.999914. At fixed dx=.1, ratios 2/4/8 give bottom
acceleration 4.409308/.230538/.005816. These measurements do not establish
arbitrary-disorder exact reproduction, convergence for all particle sequences,
or dynamic safety at large neighbor counts.

Across all 72 cases the normalized compression work residual is at most
1.03e-15; total force residual is at most 7.44e-11 and total torque at most
8.32e-13 in simulation units. Tangent density rates are exactly zero in this
arithmetic; joint rigid density rates divided by ρ are at most 1.30e-15.
Conservation and symmetry do not imply pointwise constant-pressure balance.

## Blockers and next acceptance experiments

Do not integrate this into a production solver yet. Source ownership removes
an independent boundary quadrature, but near-wall and bulk moment errors remain
under row phase/disorder. The next design decision is how to correct deficient
moments while retaining a geometry-derived velocity map and its full transpose.
A pointwise corrected pressure gradient alone may lose torque, reproduction
under rigid rotation, or density/pressure adjointness; it cannot be justified
by the current global work identity. No correction is implemented here.

A follow-up must measure regular and disordered zeroth/first moments over
multiple deterministic disorder sequences, then independently repeat geometry
finite differences, force/torque, work and null-mode checks for any correction.
It must distinguish dx refinement from increasing h/dx. Only after local
consistency is supported should it add time integration, compression/EOS
response, perturbation/pairing, and timestep/neighbor refinement without
weakening #44 criteria. Signed pressure, hydrostatic columns, multiple planes,
corners, curved surfaces and moving rigid-body coupling remain separate work.
