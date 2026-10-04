# Charged particles in uniform fields

`ChargedParticle` is a standalone nonrelativistic test charge in the XY plane.
Its prescribed electric field is in-plane and its magnetic field points along Z.
Positions, velocities, mass, charge, fields and integration use double precision;
`Vector2d` is a simple `{x, y}` coordinate value separate from the rigid engine's
single-precision `Vector2`.

```cpp
#include <physics/physics.h>
using namespace PhysicsEngine;
ChargedParticle particle({0, 0}, {2, 0}, 3, 6);
UniformElectromagneticField field{{0, 0}, 2};
particle.step(0.1, field);
auto position = particle.getPosition();
double energy = particle.getKineticEnergy();
```

Use metres, seconds, kilograms, coulombs, V/m and tesla for SI quantities. Positive
magnetic field means +Z; a positive charge initially moving along +X curves toward
-Y. `step(dt, field)` holds the field constant throughout the call. It accepts
finite `dt >= 0`; zero dt validates its inputs and preserves state. A missing field
means free motion. Mass must be finite and strictly positive; charge can be
positive, negative or zero. `setState` validates both vectors before changing either.

## Integration

The [Lorentz equation](https://openstax.org/books/university-physics-volume-2/pages/16-1-maxwells-equations-and-electromagnetic-waves)
is `dv/dt = a + omega*J*v`, with `a=q*E/m`, `omega=q*B/m` and
`J(x,y)=(y,-x)`. For `theta=omega*dt`, the implementation analytically integrates
this constant linear system:

```
v1 = cos(theta)*v0 + sin(theta)*J*v0
     + dt * [sinc(theta)*a + cosc(theta)*J*a]
x1 = x0 + dt * [sinc(theta)*v0 + cosc(theta)*J*v0]
     + dt^2 * [A(theta)*a + B(theta)*J*a]

sinc(theta) = sin(theta)/theta
cosc(theta) = (1-cos(theta))/theta
A(theta) = (1-cos(theta))/theta^2
B(theta) = (theta-sin(theta))/theta^2
```

Small-angle series avoid cancellation. Their limits are `sinc=1`, `cosc=0`,
`A=1/2`, `B=0`, recovering constant electric acceleration continuously. Exponent
scaling evaluates products such as `q*B*dt/m` without overflowing a premature
`q/m`. Complete response terms are scaled before final rounding; tiny gyro
coefficients, an underflowing electric velocity increment, or an underflowing
phase can still produce representable position or transverse-velocity changes.
Kinetic energy combines normalized velocity components before rounding, retaining
a representable total when individual component energies would underflow.
No iterative solver or timestep-dependent truncation error is introduced
for a truly constant field, apart from finite series and floating-point rounding.
Changing fields between calls gives a piecewise-constant approximation; spatially
varying fields require a different integrator.

Tests cover electric acceleration, cyclotron radius and period, both charge signs,
magnetic energy and orbit-center preservation, `E cross B / |B|^2` drift, electric
work versus kinetic-energy change, step composition/time reversal, zero/weak
fields and numeric rejection. These are tests of prescribed-field motion.

## Limits and errors

This module does not evolve fields or include particle interactions, radiation,
relativity, collisions or automatic coupling to `World`, fluids or deformable
bodies. [JavaScript bindings](webassembly.md) use the same double-precision model.
Nonrelativistic velocities are the caller's
physical regime; the API does not impose a speed-of-light constraint.

Invalid inputs throw `std::invalid_argument`. Nonrepresentable derived values
throw `std::overflow_error`; failed steps preserve position and velocity. The
energy getter separately throws if kinetic energy cannot be represented. Scaled
products improve numeric range, but arbitrary cancellation of overflowing sums
is not supported. Extremely large gyro angles amplify parameter/phase rounding;
use meaningful physical units and check precision for long trajectories.
