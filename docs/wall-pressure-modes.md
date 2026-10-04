# WCSPH wall-pressure extrapolation

`WcsphConfig::wallPressureMode` selects how sampled boundary pressure is
extrapolated from neighboring fluid particles. The default
`WcsphWallPressureMode::LegacyPositiveIncrement` preserves prior simulations.
The opt-in `SignedBodyForce` mode retains both signs of the body-force increment:

```cpp
WcsphConfig config;
config.wallPressureMode = WcsphWallPressureMode::SignedBodyForce;
WcsphSolver solver(smoothingLength, config);
```

For fluid position x_i and boundary position x_b, effective acceleration
a = externalAcceleration - boundary.acceleration, density rho_i and kernel
weight W_i, the signed mode uses:

```text
p_b = sum_i W_i [p_i - rho_i a dot (x_i-x_b)] / sum_i W_i
```

This evaluates a locally affine hydrostatic field, grad(p)=rho*a, at the
boundary. The legacy mode clips each acceleration increment below zero before
adding it to p_i. That loses the negative increment required by some side-wall
neighbors. Boundary acceleration is prescribed data, not inferred from motion.

With `clampNegativePressure=true`, signed mode clamps the **averaged** boundary
pressure below zero. It does not clamp individual extrapolated samples first.
With the flag false, signed negative boundary pressures are retained. The fluid
EOS keeps its existing negative-pressure policy. An empty support falls back to
the fluid particle's pressure; zero `pressureScale` suppresses wall pressure force
and the existing wall continuity term. Wall density quadrature is unchanged.

Signed acceleration differences, extrapolation, averaging and force products
use double precision. Internal boundary pressure may exceed float range if the
resulting stored force remains finite. A force outside finite `Vector2` range
throws `std::overflow_error` before that force is assigned. This does **not**
make all of `prepare()` or `step()` transactional: earlier particle updates may
already have happened. The surrounding float particle/grid/kernel limitations
still apply. Unknown enum values are rejected during configuration validation.
Adding this field changes the C++ configuration layout; rebuild native consumers.

## Measured effect and limits

The [sampled-wall audit](fluid-wall-audit.md) reconstructs both modes independently.
Run the identical 84-state matrix with:

```sh
build/fluid_wall_diagnostic --output wall-legacy.json
build/fluid_wall_diagnostic --signed-pressure --output wall-signed.json
```

The output labels the selected mode in `settings.wallPressureMode`. `--quick`
selects 12 prepared states. The constant-pressure phase controls use zero body
force, so mode selection does not affect them. Counterfactual fields retain
their original names; signed counterfactual values agree with actual signed
mode up to float force storage and accumulation error.

Windows Clang 23.1.1 Release, base 7c0df78 plus this implementation, reproduces
every legacy case value from the original #69 report. The signed report's maximum
relative independent force reconstruction error is 2.88e-6. For the initialized
Tait-EOS column at spacing .1:

| Kernel | h/dx | Legacy acceleration RMS | Signed acceleration RMS |
|---|---:|---:|---:|
| Poly6/Spiky | 2 | 2.57899 | 2.61741 |
| Poly6/Spiky | 2.5 | 1.81971 | 1.76799 |
| Poly6/Spiky | 4 | 1.68449 | 1.59254 |
| Cubic | 2 | 2.33548 | 2.39033 |
| Cubic | 2.5 | 2.41757 | 2.27455 |
| Cubic | 4 | 1.86553 | 1.78556 |

The [complete signed-mode report](data/fluid-wall-signed-582bf00-clang23.json)
records all refinement states and pressure-work measurements.

At cubic h/dx=2.5, vertical force residual/weight changes from .006489 to
.006354; bottom-interior acceleration RMS remains 2.90344. Improvements are
not uniform across neighbor ratios. These are prepared-state observations,
not stable time-integrated hydrostatic columns.

Signed extrapolation does not fix sampling phase/disorder errors, change the
radial wall continuity model or track the virtual reservoir's pressure-work
exchange. All nine #44 acceptance failures retain their thresholds. This option
is an explicit pressure-extrapolation experiment, not a general boundary solver.
