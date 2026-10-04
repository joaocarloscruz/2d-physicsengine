#include "physics/core/charged_particle.h"

#include <cmath>
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
double product(double a, double b, double c, double divisor = 1.0) {
    if (a == 0 || b == 0 || c == 0) return 0;
    int ea, eb, ec, ed;
    const double ma = std::frexp(a, &ea), mb = std::frexp(b, &eb);
    const double mc = std::frexp(c, &ec), md = std::frexp(divisor, &ed);
    return checked(std::scalbn(((ma * mb) * mc) / md, ea + eb + ec - ed));
}
// sin(t)/t, (1-cos(t))/t and their time integrals. Series avoid
// subtractive cancellation and give the exact continuous zero-field limit.
struct Coefficients { double sine, cosine, sinc, cosc, a, b; };
Coefficients coefficients(double t) {
    const double sine = std::sin(t), cosine = std::cos(t);
    if (std::abs(t) < 0.01) {
        const double t2 = t * t;
        return {sine, cosine,
            1 + t2 * (-1.0/6 + t2 * (1.0/120 - t2/5040)),
            t * (0.5 + t2 * (-1.0/24 + t2 * (1.0/720 - t2/40320))),
            0.5 + t2 * (-1.0/24 + t2 * (1.0/720 - t2/40320)),
            t * (1.0/6 + t2 * (-1.0/120 + t2 * (1.0/5040 - t2/362880)))};
    }
    const double halfSine = std::sin(t * 0.5);
    const double cosc = 2 * halfSine * halfSine / t;
    const double sinc = sine / t;
    return {sine, cosine, sinc, cosc, cosc / t, (1 - sinc) / t};
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
    return checked(product(mass_, velocity_.x, velocity_.x, 2.0)
        + product(mass_, velocity_.y, velocity_.y, 2.0));
}
void ChargedParticle::step(double dt, const UniformElectromagneticField& field) {
    if (!std::isfinite(dt) || dt < 0 || !std::isfinite(field.magnetic))
        throw std::invalid_argument("Charged-particle timestep and magnetic field must be finite; dt >= 0");
    validate(field.electric);
    if (dt == 0) return;
    // An exactly neutral particle is ballistic even for very large fields.
    double theta = 0, ux = 0, uy = 0;
    if (charge_ != 0 && (field.magnetic != 0 || field.electric.x != 0 || field.electric.y != 0)) {
        theta = product(charge_, field.magnetic, dt, mass_);
        ux = product(charge_, field.electric.x, dt, mass_);
        uy = product(charge_, field.electric.y, dt, mass_);
    }
    const auto c = coefficients(theta);
    // J(x,y)=(y,-x): v cross (+Z) rotates a positive charge clockwise.
    const Vector2d nextVelocity{
        checked(c.cosine * velocity_.x + c.sine * velocity_.y + c.sinc * ux + c.cosc * uy),
        checked(c.cosine * velocity_.y - c.sine * velocity_.x + c.sinc * uy - c.cosc * ux)};
    const Vector2d nextPosition{
        checked(position_.x + checked(dt * (c.sinc * velocity_.x + c.cosc * velocity_.y + c.a * ux + c.b * uy))),
        checked(position_.y + checked(dt * (c.sinc * velocity_.y - c.cosc * velocity_.x + c.a * uy - c.b * ux)))};
    position_ = nextPosition;
    velocity_ = nextVelocity;
}
}
