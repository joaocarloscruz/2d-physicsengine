# Fluid physics validation matrix

This matrix separates numerical consistency, boundary behavior, continuum
benchmarks, and rigid coupling. A visually plausible result is not an oracle.
Every scenario records dimensionless errors or conserved quantities and runs
headlessly with deterministic initial data.

WCSPH acceptance uses the common requirement that artificial sound speed remain
at least ten times the maximum flow speed, limiting density variation to about
one percent, as documented in the
[DualSPHysics formulation](https://github.com/DualSPHysics/DualSPHysics/wiki/3.-SPH-formulation#34-equation-of-state).
Its variable timestep also includes force-per-unit-mass, Courant, and viscous
limits. Boundary expectations follow the local force-balance approach of
[Adami, Hu, and Adams](https://doi.org/10.1016/j.jcp.2012.05.005), while the
longer-term benchmark set follows the categories maintained by
[SPHERIC](https://www.spheric-sph.org/validation-tests).

| Risk | Scenario | Test level | Primary oracle |
|---|---|---|---|
| Kernel inconsistency | translated uniform lattice | discrete operator | interior density error and translation invariance |
| Incorrect pressure sign | compressed symmetric pair | force unit | repulsion direction and zero net internal force |
| Frame dependence | uniform translating lattice | solver integration | Galilean-equivalent relative state |
| Pressure explosion | highly compressed pair | timestep | selected step does not exceed force criterion |
| Free-surface deficiency | open lattice top rows | characterization | quantify truncated support separately from bulk density |
| Wall density bias | uniform lattice beside flat wall | boundary operator | wall density agrees with bulk within 5% |
| Corner overpopulation | uniform lattice in 90-degree corner | boundary operator | corner density agrees with bulk within 5% |
| Wall adhesion | tangential particle motion | contact unit | zero-friction wall preserves tangential velocity |
| Wall rebound | resting/approaching particles | contact unit | zero restitution produces no outward launch |
| Leakage | long static tank | integration | all particle centers remain in valid domain |
| Hydrostatic imbalance | still-water column | continuum | low velocity, near-linear pressure, controlled bulk density |
| Energy injection | damped sloshing | continuum | mechanical energy envelope does not grow |
| Disordered particles | perturbed lattice | stability | bounded density percentiles and no pair collapse |
| Resolution dependence | hydrostatic and dam break at 3 spacings | convergence | observable error decreases with spacing |
| Timestep dependence | same scene at dt, dt/2, dt/4 | convergence | state differences decrease under refinement |
| Dam-break dynamics | SPHERIC-style 2-D column collapse | continuum | front-position curve, mass, bounded density |
| Incorrect buoyancy | fully/partly immersed circle | coupling | Archimedes force magnitude and direction |
| Neutral buoyancy drift | body density equals fluid | coupling | bounded heave and vertical velocity |
| Coupled momentum loss | pressure, viscosity, contact | coupling | total dynamic momentum within documented tolerance |
| Boundary sticking on bodies | tangential flow past rigid surface | coupling | no artificial tangential arrest in free-slip mode |
| Long-run settling | light and heavy bodies in tank | system | partial flotation and floor settling without jitter |
| Performance collapse | 1k/3k/10k scenarios | non-functional | median/p95 simulation time and local work counts |

The first implementation pass intentionally allows validation tests to fail.
Each failure must be localized before changing a numerical model. Thresholds
that represent engineering targets are not weakened to match current output.

## Baseline findings (2026-07-19)

The deterministic validation executable currently contains 26 fluid-validation
cases with 66 assertions. After the first corrective pass, 20 cases and 60
assertions pass; the remaining six failures are retained as characterization
tests for unresolved physics.

| Area | Measured result | Status / interpretation |
|---|---:|---|
| Square-lattice density, h/dx=2 | 1.0146127 rho0 | fails uncalibrated 1% target |
| Calibrated rest block, 0.1 s | 5.25e-5 m/s peak | calibration removes nominal self-expansion |
| Nominal rest block, 0.1 s | 0.2272 m/s peak | fails; spatial quadrature bias dominates |
| Force-aware timestep | 1.47797e-5 s | fixed; equals analytical force limit |
| Default wall tangential retention | 1.0 after 120 contacts | fixed; default is free-slip |
| Friction subdivision | 0.995 vs 0.995 | fixed; tied to total normal impulse |
| Flat / corner wall density | -0.28% / +2.96% vs bulk | passes 5% target |
| Curved wall density | +0.70% vs bulk | passes 5% target |
| Initial hydrostatic pressure RMSE | 54.84% normalized | fails; column starts in free fall |
| Initial bulk acceleration | 9.8098 m/s2 RMS | fails; no initial pressure support |
| Hydrostatic run, 0.5 s | 61.42% density error, 16.08 m/s | fails after mass calibration |
| Timestep refinement, 0.2 s | 4.857 vs 4.838 m/s | formulation error persists at dt/4 |
| Positional disorder, 0.2 s | no change from 2.828 mm RMS | fails; no regularization below rho0 |
| Analytical circle buoyancy | 7704.75 vs 7704.76 N | passes; pressure integration is correct |

Unresolved work is therefore ordered as: hydrostatic initialization and wall
reaction, density diffusion/particle shifting, hydrostatic and sloshing
convergence, then coupled floating/settling and dam-break benchmarks.
