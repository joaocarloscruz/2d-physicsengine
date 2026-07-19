#ifndef FLUID_BOUNDARY_H
#define FLUID_BOUNDARY_H

#include "fluid_particle.h"

#include <cstddef>
#include <vector>

namespace PhysicsEngine {

struct FluidBoundarySettings {
    float particleRadius = 0.05f;
    float restitution = 0.0f;
    float friction = 0.05f;

    void Validate() const;
};

struct FluidBoundaryCorrection {
    bool corrected = false;
    float penetration = 0.0f;
};

struct FluidBoundaryStatistics {
    std::size_t correctedParticleCount = 0;
    float maximumPenetration = 0.0f;
};

class IFluidContainer {
public:
    virtual ~IFluidContainer() = default;

    virtual FluidBoundaryCorrection enforce(FluidParticle& particle) const = 0;
    virtual bool contains(const Vector2& position) const = 0;
};

class FluidCircleContainer : public IFluidContainer {
public:
    FluidCircleContainer(
        const Vector2& center,
        float radius,
        const FluidBoundarySettings& settings = FluidBoundarySettings{}
    );

    FluidBoundaryCorrection enforce(FluidParticle& particle) const override;
    bool contains(const Vector2& position) const override;

private:
    Vector2 center;
    float radius;
    FluidBoundarySettings settings;
};

class FluidConvexPolygonContainer : public IFluidContainer {
public:
    FluidConvexPolygonContainer(
        std::vector<Vector2> vertices,
        const FluidBoundarySettings& settings = FluidBoundarySettings{}
    );

    FluidBoundaryCorrection enforce(FluidParticle& particle) const override;
    bool contains(const Vector2& position) const override;

private:
    std::vector<Vector2> vertices;
    std::vector<Vector2> inwardNormals;
    FluidBoundarySettings settings;
};

FluidBoundaryStatistics EnforceFluidBoundary(
    const IFluidContainer& boundary,
    std::vector<FluidParticle>& particles
);

}

#endif // FLUID_BOUNDARY_H
