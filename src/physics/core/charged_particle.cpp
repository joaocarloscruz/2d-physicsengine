#include "physics/core/charged_particle.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace PhysicsEngine {
namespace {
void validate(Vector2d v) {
    if (!std::isfinite(v.x) || !std::isfinite(v.y))
        throw std::invalid_argument("Charged-particle coordinates and fields must be finite");
}
double checked(double x) {
    if (!std::isfinite(x)) throw std::overflow_error("Charged-particle arithmetic overflow");
    return x;
}
// Keep finite products even when an arbitrary evaluation order would overflow
// (for example microscopic mass and charge with a very short timestep).
double product(std::initializer_list<double> factors,
               std::initializer_list<double> divisors = {}) {
    double mantissa = 1.0;
    int exponent = 0;
    // At most eight factors/three divisors below: mantissa arithmetic remains
    // normal even when the complete product has an extreme binary exponent.
    for (const double factor : factors) {
        if (factor == 0) return 0;
        int e;
        mantissa *= std::frexp(factor, &e);
        exponent += e;
    }
    for (const double divisor : divisors) {
        int e;
        mantissa /= std::frexp(divisor, &e);
        exponent -= e;
    }
    return checked(std::scalbn(mantissa, exponent));
}
// sin(t)/t, (1-cos(t))/t and their time integrals. Series avoid
// subtractive cancellation and give the exact continuous zero-field limit.
// Keep cosc=theta*a and B=theta*bOverTheta factored. Materializing those tiny
// coefficients (or A~1/theta^2 at huge angles) can erase finite response terms.
struct Coefficients { double sine, cosine, sinc, a, bOverTheta, oneMinusCosine; bool small; };
Coefficients coefficients(double t) {
    const double sine = std::sin(t), cosine = std::cos(t);
    if (std::abs(t) < 0.01) {
        const double t2 = t * t;
        return {sine, cosine,
            1 + t2 * (-1.0/6 + t2 * (1.0/120 - t2/5040)),
            0.5 + t2 * (-1.0/24 + t2 * (1.0/720 - t2/40320)),
            1.0/6 + t2 * (-1.0/120 + t2 * (1.0/5040 - t2/362880)), 0.0, true};
    }
    const double halfSine = std::sin(t * 0.5);
    const double sinc = sine / t;
    return {sine, cosine, sinc, 0.0, 0.0, 2 * halfSine * halfSine, false};
}
}

ChargedParticle::ChargedParticle(Vector2d position, Vector2d velocity, double mass, double charge)
    : position_(position), velocity_(velocity), mass_(mass), charge_(charge) {
    validate(position); validate(velocity);
    if (!std::isfinite(mass) || mass <= 0 || !std::isfinite(charge))
        throw std::invalid_argument("Charged-particle mass must be finite and positive; charge must be finite");
}
void ChargedParticle::setState(Vector2d position, Vector2d velocity) {
    validate(position); validate(velocity);
    position_ = position;
    velocity_ = velocity;
}
double ChargedParticle::getKineticEnergy() const {
    const double scale = std::max(std::abs(velocity_.x), std::abs(velocity_.y));
    if (scale == 0) return 0;
    const double x = velocity_.x / scale, y = velocity_.y / scale;
    // Combine components before rounding the final energy. Two half-subnormal
    // component energies can together be representable, even if each rounds zero.
    return product({mass_, scale, scale, x*x + y*y}, {2.0});
}
void ChargedParticle::step(double dt, const UniformElectromagneticField& field) {
    if (!std::isfinite(dt) || dt < 0 || !std::isfinite(field.magnetic))
        throw std::invalid_argument("Charged-particle timestep and magnetic field must be finite; dt >= 0");
    validate(field.electric);
    if (dt == 0) return;
    // An exactly neutral particle is ballistic even for very large fields.
    double theta = 0;
    if (charge_ != 0 && (field.magnetic != 0 || field.electric.x != 0 || field.electric.y != 0)) {
        theta = product({charge_, field.magnetic, dt}, {mass_});
    }
    const auto c = coefficients(theta);
    // J(x,y)=(y,-x): v cross (+Z) rotates a positive charge clockwise.
    Vector2d nextVelocity, displacement;
    if (c.small) {
        // Evaluate theta factors from the original inputs as well: theta may
        // underflow while theta*velocity or theta*electric impulse is finite.
        const auto rotation = [&](double velocity) {
            return product({charge_, field.magnetic, dt, c.sinc, velocity}, {mass_});
        };
        const auto impulse = [&](double electric) {
            return product({charge_, electric, dt, c.sinc}, {mass_});
        };
        const auto crossImpulse = [&](double electric) {
            return product({charge_, charge_, field.magnetic, electric, dt, dt, c.a}, {mass_, mass_});
        };
        const auto crossDisplacement = [&](double velocity) {
            return product({charge_, field.magnetic, dt, dt, c.a, velocity}, {mass_});
        };
        const auto electricDisplacement = [&](double electric) {
            return product({charge_, electric, dt, dt, c.a}, {mass_});
        };
        const auto crossElectricDisplacement = [&](double electric) {
            return product({charge_, charge_, field.magnetic, electric, dt, dt, dt, c.bOverTheta}, {mass_, mass_});
        };
        nextVelocity = {
            checked(c.cosine * velocity_.x + rotation(velocity_.y) + impulse(field.electric.x) + crossImpulse(field.electric.y)),
            checked(c.cosine * velocity_.y - rotation(velocity_.x) + impulse(field.electric.y) - crossImpulse(field.electric.x))};
        displacement = {
            checked(product({dt, c.sinc, velocity_.x}) + crossDisplacement(velocity_.y)
                + electricDisplacement(field.electric.x) + crossElectricDisplacement(field.electric.y)),
            checked(product({dt, c.sinc, velocity_.y}) - crossDisplacement(velocity_.x)
                + electricDisplacement(field.electric.y) - crossElectricDisplacement(field.electric.x))};
    } else {
        const auto impulse = [&](double electric, double numerator) {
            return product({charge_, electric, dt, numerator}, {mass_, theta});
        };
        const auto electricDisplacement = [&](double electric) {
            return product({charge_, electric, dt, dt, c.oneMinusCosine}, {mass_, theta, theta});
        };
        const auto crossElectricDisplacement = [&](double electric) {
            return product({charge_, electric, dt, dt, 1 - c.sinc}, {mass_, theta});
        };
        nextVelocity = {
            checked(c.cosine * velocity_.x + c.sine * velocity_.y + impulse(field.electric.x, c.sine) + impulse(field.electric.y, c.oneMinusCosine)),
            checked(c.cosine * velocity_.y - c.sine * velocity_.x + impulse(field.electric.y, c.sine) - impulse(field.electric.x, c.oneMinusCosine))};
        displacement = {
            checked(product({dt, c.sine, velocity_.x}, {theta}) + product({dt, c.oneMinusCosine, velocity_.y}, {theta})
                + electricDisplacement(field.electric.x) + crossElectricDisplacement(field.electric.y)),
            checked(product({dt, c.sine, velocity_.y}, {theta}) - product({dt, c.oneMinusCosine, velocity_.x}, {theta})
                + electricDisplacement(field.electric.y) - crossElectricDisplacement(field.electric.x))};
    }
    const Vector2d nextPosition{checked(position_.x + displacement.x), checked(position_.y + displacement.y)};
    position_ = nextPosition;
    velocity_ = nextVelocity;
}
}
