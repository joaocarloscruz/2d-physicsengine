#ifndef SPH_KERNELS_H
#define SPH_KERNELS_H

#include "../../math/vector2.h"

namespace PhysicsEngine {

enum class SphKernelFamily {
    Poly6Spiky, // Legacy poly6 density and spiky pressure derivative.
    CubicSpline // Matched normalized cubic weight/gradient, full support h.
};

class SphKernels2D {
public:
    static void ValidateFamily(SphKernelFamily family);
    // Corrects rho * spacing^2 particle masses for the discrete density sum
    // of an infinite square lattice at the requested resolution.
    static float SquareLatticeMassScale(
        float spacing,
        float smoothingLength
    );
    static float SquareLatticeMassScale(
        float spacing,
        float smoothingLength,
        SphKernelFamily family
    );
    static float DensityWeight(
        const Vector2& displacement,
        float smoothingLength
    );
    static float DensityWeight(
        const Vector2& displacement,
        float smoothingLength,
        SphKernelFamily family
    );
    static float PressureWeight(
        const Vector2& displacement,
        float smoothingLength
    );
    static float PressureWeight(
        const Vector2& displacement,
        float smoothingLength,
        SphKernelFamily family
    );
    static Vector2 PressureGradient(
        const Vector2& displacement,
        float smoothingLength
    );
    static Vector2 PressureGradient(
        const Vector2& displacement,
        float smoothingLength,
        SphKernelFamily family
    );
    // Independent Muller viscosity operator; unchanged by the kernel family.
    static float ViscosityLaplacian(
        const Vector2& displacement,
        float smoothingLength
    );
};

}

#endif // SPH_KERNELS_H
