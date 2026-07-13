#include "physics/core/particles/particle_system.h"

#include <stdexcept>

namespace PhysicsEngine {

void ParticleSystem::reserve(std::size_t capacity) {
    particles.reserve(capacity);
}

std::size_t ParticleSystem::addParticle(
    const Vector2& position,
    const Vector2& velocity,
    float mass
) {
    particles.emplace_back(position, velocity, mass);
    return particles.size() - 1;
}

void ParticleSystem::removeParticle(std::size_t index) {
    if (index >= particles.size()) {
        throw std::out_of_range("Particle index is out of range.");
    }
    particles.erase(particles.begin() + static_cast<std::ptrdiff_t>(index));
}

void ParticleSystem::applyForce(std::size_t index, const Vector2& force) {
    if (index >= particles.size()) {
        throw std::out_of_range("Particle index is out of range.");
    }
    particles[index].ApplyForce(force);
}

void ParticleSystem::clear() {
    particles.clear();
}

void ParticleSystem::step(float deltaTime) {
    for (Particle& particle : particles) {
        particle.ApplyForce(uniformAcceleration * particle.mass);
        particle.Integrate(deltaTime);
    }
}

void ParticleSystem::setUniformAcceleration(const Vector2& acceleration) {
    uniformAcceleration = acceleration;
}

Vector2 ParticleSystem::getUniformAcceleration() const {
    return uniformAcceleration;
}

std::size_t ParticleSystem::size() const {
    return particles.size();
}

bool ParticleSystem::empty() const {
    return particles.empty();
}

std::vector<Particle>& ParticleSystem::getParticles() {
    return particles;
}

const std::vector<Particle>& ParticleSystem::getParticles() const {
    return particles;
}

}
