#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

#include <cstddef>
#include <vector>

#include "particle.h"

namespace PhysicsEngine {

class ParticleSystem {
public:
    void reserve(std::size_t capacity);
    std::size_t addParticle(
        const Vector2& position,
        const Vector2& velocity = Vector2(),
        float mass = 1.0f
    );
    void removeParticle(std::size_t index);
    void applyForce(std::size_t index, const Vector2& force);
    void clear();
    void step(float deltaTime);

    void setUniformAcceleration(const Vector2& acceleration);
    Vector2 getUniformAcceleration() const;

    std::size_t size() const;
    bool empty() const;
    std::vector<Particle>& getParticles();
    const std::vector<Particle>& getParticles() const;

private:
    std::vector<Particle> particles;
    Vector2 uniformAcceleration;
};

}

#endif // PARTICLE_SYSTEM_H
