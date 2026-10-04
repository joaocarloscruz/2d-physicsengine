# Wendland C2: matched pressure work and measured limits

The optional `SphKernelFamily::WendlandC2` implements the normalized 2D
[weight and gradient](sph-kernels.md#optional-wendland-c2-family).
It preserves the fixed-support density/pressure energy relationship, but does
**not** solve the existing disorder target. The default remains `Poly6Spiky`;
all nine legacy expected failures and their original thresholds remain.

```sh
cmake --build build --parallel 2
build/run_tests "[wendland],[variable-support],[dfsph]"
build/fluid_disorder_diagnostic --include-wendland --output build/wendland-full.json
build/fluid_disorder_diagnostic --quick --include-wendland --output build/wendland-quick.json
ctest --test-dir build --output-on-failure
```

Append `.exe` on Windows. The default diagnostic retains the original two-family
comparison; `--include-wendland` appends a third family. `fixtureBaseCommit`
identifies the original fixture, not the current implementation. The
[full report](data/fluid-wendland-bdd513e-clang23.json) records implementation
`bdd513e`, Windows x86_64 Clang 23.1.1 Release. It contains 36 infinite-lattice
rows, 24 original-input trajectories, 15 separately labeled controls and three
pressure-work checks, totaling 7,568 actual fluid substeps. Every legacy/cubic
numeric row exactly matches the earlier recorded report. All JSON numeric values
were checked for finiteness; the archive contains the unfavorable results too.

## The same input does not imply the same density bias

The original 21x21 fixture uses spacing `.1f`, full support `.2f`, rest density
1000 and caller mass scale `1/1.014612675f`. At support/spacing 2, the infinite
Wendland lattice has nominal density ratio 1.03760178699; the checkerboard
perturbation raises this to 1.03871008584. With the **original** caller mass,
the bulk ratio is about 1.02375036. Thus the clamped Wendland case starts with
positive pressure and expands; it cannot be compared to the stationary
underdense legacy/cubic case as if the initial thermodynamic states matched.

All rows below use 192 outer steps over the same stored-float duration near .2,
with the same geometry, viscosity, loads and material inputs. Initial RMS
distance from the labeled lattice is .00282840523. Distances are metres; minimum
separation is divided by the original spacing. Energy/separation extrema are
sampled at the initial state and eight equally spaced outer-step endpoints.

| Family | Clamp negative pressure | Final RMS distance | Sampled minimum separation / spacing | Peak speed |
| --- | --- | ---: | ---: | ---: |
| Legacy | Yes | .00282840523 | .960832498 | 0 |
| Cubic | Yes | .00282840523 | .960832498 | 0 |
| Wendland C2 | Yes | .0583281792 | .802694020 | .443003407 |
| Legacy | No | .183441660 | .00261592896 | 8.89709183 |
| Cubic | No | .0704381479 | .0331615092 | 4.28038218 |
| Wendland C2 | No | .0675368794 | .187045736 | 3.13043543 |

No Wendland original-input trajectory at 48/96/192/384 outer steps meets the
original 10% RMS healing target. The clamped final RMS decreases only from
.0585951 to .0582349 over that refinement. Finite motion, lower peak speed in
one unclamped case or larger separation alone does not establish accurate
free-surface equilibrium or absence of tensile/pairing instability.

An explicitly labeled **different-input** control uses Wendland's square-lattice
mass scale .963760853. With the clamp it ends at RMS .00326467162 and peak speed
.0193134726, still worse than its initial RMS and still failing the healing
target. Without the clamp it ends at RMS .0575991429 and sampled separation
.0427268343 of a spacing. Keeping the original mass but increasing support/spacing
to 4 makes the clamped case stationary and underdense. None of these controls
changes a mass automatically or justifies changing the original target.

## What the independent checks establish

For the smooth compressed pressure-work fixture, Wendland mechanical work is
1648.86323226 and the actual summation EOS energy rate is -1648.86323226, with
residual -4.55e-13. Independent centered energy derivatives at `.001/.0005/.00025`
have errors `.00120816506/.000302041169/.0000755100270`, decreasing by four.
Unequal-support tests separately differentiate the full EOS energy with respect
to each coordinate, retaining mass, rest density and support. They check central
pair forces, torque, work, clamped/unclamped branches and repeated-step momentum.

The checkerboard's linear density symbol remains zero by centrosymmetry for
Wendland as for the other kernels. A matched gradient fixes the pressure-work
mismatch; it does not remove that null mode. Independent kernel moment, scale,
range, wall/solver dispatch and DFSPH residual tests pass. The 13 focused cases
pass 109,358 assertions in both Release and ASan/UBSan builds; the full native
suite has 639 cases, 630 passed and nine expected failures. These checks establish
the documented formulation and operating cases, not a general SPH consistency fix.
