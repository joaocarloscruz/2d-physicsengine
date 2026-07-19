#ifndef SPH_KERNELS_H
#define SPH_KERNELS_H

#include "../../math/vector2.h"

namespace PhysicsEngine {

class SphKernels2D {
public:
    // Corrects rho * spacing^2 particle masses for the discrete density sum
    // of an infinite square lattice at the requested resolution.
    static float SquareLatticeMassScale(
        float spacing,
        float smoothingLength
    );
    static float DensityWeight(
        const Vector2& displacement,
        float smoothingLength
    );
    static float PressureWeight(
        const Vector2& displacement,
        float smoothingLength
    );
    static Vector2 PressureGradient(
        const Vector2& displacement,
        float smoothingLength
    );
    static float ViscosityLaplacian(
        const Vector2& displacement,
        float smoothingLength
    );
};

}

#endif // SPH_KERNELS_H
