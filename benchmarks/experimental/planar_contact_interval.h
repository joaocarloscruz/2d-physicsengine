#pragma once

// Diagnostic-only constant-load, nonrotating planar support. Not installed.
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace PlanarContactInterval {
enum class Status { Complete, NeedsImpact, UnsupportedWrench };
enum class Mode { Free, Sticking, Sliding };
struct Input {
    double mass = 1, tangentPosition = 0, tangentVelocity = 0;
    double gap = 0, normalVelocity = 0;
    double forceT = 0, forceN = -8, torque = 0;
    double staticFriction = .25, dynamicFriction = .25;
    // r_i = s_i*t - leverDepth*n, cross(t,n)=1.
    double left = -.5, right = .5, leverDepth = .5, duration = .125;
};
struct Interval {
    Mode mode;
    double duration, tangentDistance, normalDistance;
    double initialTangentVelocity, finalTangentVelocity, initialNormalVelocity, finalNormalVelocity;
    double normalForce, tangentForce, leftNormal, rightNormal, leftTangent, rightTangent;
    double externalWork, frictionWork, kineticChange;
};
struct Result {
    Status status = Status::Complete;
    double elapsed = 0, tangentPosition = 0, tangentVelocity = 0, gap = 0, normalVelocity = 0;
    double normalImpulse = 0, tangentImpulse = 0, spinImpulse = 0;
    double externalWork = 0, frictionWork = 0, kineticChange = 0;
    double eventGapCorrection = 0;
    std::vector<Interval> intervals;
};
namespace Detail {
inline double Finite(double x) {
    if (!std::isfinite(x))
        throw std::overflow_error("Planar interval derived range exceeded");
    return x;
}
inline double Product(double x, double y) {
    const double r = Finite(x * y);
    if (x != 0 && y != 0 && r == 0)
        throw std::overflow_error("Planar interval product underflow");
    return r;
}
inline double Ratio(double x, double y) {
    const double r = Finite(x / y);
    if (x != 0 && r == 0)
        throw std::overflow_error("Planar interval ratio underflow");
    return r;
}
// The earliest strictly positive closing root. All intermediate range is
// checked; this diagnostic explicitly rejects extreme unresolved quadratics.
inline double ClosingTime(double gap, double velocity, double acceleration) {
    if (gap == 0) {
        if (velocity < 0 || (velocity == 0 && acceleration < 0))
            return 0;
        if (velocity > 0 && acceleration < 0)
            return Product(-2, Ratio(velocity, acceleration));
        return -1;
    }
    if (acceleration == 0)
        return velocity < 0 ? Ratio(-gap, velocity) : -1;
    const double discriminant =
        Finite(Product(velocity, velocity) - Product(Product(2, acceleration), gap));
    if (discriminant <= 0 || (acceleration > 0 && velocity >= 0))
        return -1;
    const double root = std::sqrt(discriminant);
    // A separating trajectory with inward acceleration returns through the
    // far root. sqrt(D)-v can round to zero for a tiny positive starting gap.
    if (velocity > 0 && acceleration < 0)
        return Ratio(Finite(-velocity - root), acceleration);
    const double denominator = Finite(root - velocity);
    if (denominator <= 0)
        return -1;
    return Ratio(gap, Product(.5, denominator));
}
inline bool Wrench(const Input &in, double normal, double tangent, double &left, double &right) {
    const double moment = Finite(-in.torque - Product(in.leverDepth, tangent));
    if (normal == 0) {
        left = right = 0;
        return moment == 0 && tangent == 0;
    }
    const double center = Ratio(moment, normal);
    if (center < in.left || center > in.right)
        return false;
    const double span = Finite(in.right - in.left);
    left = Product(normal, Ratio(Finite(in.right - center), span));
    right = Product(normal, Ratio(Finite(center - in.left), span));
    return true;
}
inline void Append(const Input &in, Result &r, Mode mode, double h, double normal, double tangent,
                   double left, double right, double tangentAcceleration, double normalAcceleration,
                   bool stop) {
    const double vt = r.tangentVelocity, vn = r.normalVelocity;
    const double nextT = stop ? 0 : Finite(std::fma(tangentAcceleration, h, vt));
    const double nextN = Finite(std::fma(normalAcceleration, h, vn));
    const double dx = Product(Finite(Product(vt, .5) + Product(nextT, .5)), h);
    const double dy = Product(Finite(Product(vn, .5) + Product(nextN, .5)), h);
    const double external = Finite(Product(in.forceT, dx) + Product(in.forceN, dy));
    const double friction = Product(tangent, dx);
    // Independent endpoint kinetic accounting (not a balanced work residual).
    const double kinetic =
        Product(Product(in.mass, .5), Finite(Product(Finite(nextT - vt), Finite(nextT + vt)) +
                                             Product(Finite(nextN - vn), Finite(nextN + vn))));
    const double shearRatio = normal == 0 ? 0 : Ratio(tangent, normal);
    const double leftTangent = Product(shearRatio, left), rightTangent = Product(shearRatio, right);
    r.intervals.push_back({mode, h, dx, dy, vt, nextT, vn, nextN, normal, tangent, left, right,
                           leftTangent, rightTangent, external, friction, kinetic});
    r.tangentPosition = Finite(r.tangentPosition + dx);
    r.gap = Finite(r.gap + dy);
    r.tangentVelocity = nextT;
    r.normalVelocity = nextN;
    r.elapsed = Finite(r.elapsed + h);
    r.normalImpulse = Finite(r.normalImpulse + Product(normal, h));
    r.tangentImpulse = Finite(r.tangentImpulse + Product(tangent, h));
    r.spinImpulse = Finite(r.spinImpulse + Product(-in.torque, mode == Mode::Free ? 0 : h));
    r.externalWork = Finite(r.externalWork + external);
    r.frictionWork = Finite(r.frictionWork + friction);
    r.kineticChange = Finite(r.kineticChange + kinetic);
}
} // namespace Detail
inline Result Advance(const Input &in) {
    using namespace Detail;
    for (double x : {in.mass, in.tangentPosition, in.tangentVelocity, in.gap, in.normalVelocity,
                     in.forceT, in.forceN, in.torque, in.staticFriction, in.dynamicFriction,
                     in.left, in.right, in.leverDepth, in.duration})
        if (!std::isfinite(x))
            throw std::invalid_argument("Planar interval inputs must be finite");
    if (in.mass <= 0 || in.gap < 0 || in.duration < 0 || in.leverDepth < 0 || in.left >= in.right ||
        in.dynamicFriction < 0 || in.staticFriction < in.dynamicFriction)
        throw std::invalid_argument("Invalid planar interval geometry/material/time");
    Result r;
    r.tangentPosition = in.tangentPosition;
    r.tangentVelocity = in.tangentVelocity;
    r.gap = in.gap;
    r.normalVelocity = in.normalVelocity;
    if (in.duration == 0)
        return r;
    const double at = Ratio(in.forceT, in.mass), an = Ratio(in.forceN, in.mass);
    const bool supported = in.gap == 0 && in.normalVelocity == 0 && in.forceN <= 0;
    if (!supported) {
        // Geometry assumes a nonrotating body; do not silently freeze a free
        // body with applied torque or model an unrequested impact response.
        if (in.torque != 0) {
            r.status = Status::UnsupportedWrench;
            return r;
        }
        const double onset = ClosingTime(in.gap, in.normalVelocity, an);
        const bool hit = onset >= 0 && onset <= in.duration;
        const double h = hit ? onset : in.duration;
        if (h > 0)
            Append(in, r, Mode::Free, h, 0, 0, 0, 0, at, an, false);
        if ((!hit && r.gap < 0) || (hit && r.normalVelocity > 0))
            throw std::overflow_error("Unresolved planar closing event");
        if (hit) {
            r.eventGapCorrection = -r.gap;
            r.gap = 0;
            r.status = Status::NeedsImpact;
        }
        return r;
    }
    const double normal = -in.forceN;
    const double staticLimit = Product(in.staticFriction, normal);
    const double kineticLimit = Product(in.dynamicFriction, normal);
    double remaining = in.duration;
    // Constant load and mu_s>=mu_k allow at most one stop and one restart.
    for (unsigned i = 0; i < 2 && remaining > 0; ++i) {
        const bool stick = r.tangentVelocity == 0 && std::abs(in.forceT) <= staticLimit;
        const double direction = r.tangentVelocity == 0 ? std::copysign(1, in.forceT)
                                                        : std::copysign(1, r.tangentVelocity);
        const double tangent = stick ? -in.forceT : Product(-direction, kineticLimit);
        double left = 0, right = 0;
        if (!Wrench(in, normal, tangent, left, right)) {
            r.status = Status::UnsupportedWrench;
            return r;
        }
        const double acceleration = stick ? 0 : Ratio(Finite(in.forceT + tangent), in.mass);
        double h = remaining;
        bool stop = false;
        if (r.tangentVelocity != 0 && acceleration != 0 &&
            std::signbit(r.tangentVelocity) != std::signbit(acceleration)) {
            const double stopping = Ratio(-r.tangentVelocity, acceleration);
            if (stopping <= remaining) {
                h = stopping;
                stop = true;
            }
        }
        Append(in, r, stick ? Mode::Sticking : Mode::Sliding, h, normal, tangent, left, right,
               acceleration, 0, stop);
        remaining = Finite(remaining - h);
    }
    if (remaining != 0)
        throw std::logic_error("Planar interval event budget exceeded");
    r.elapsed = in.duration;
    return r;
}
} // namespace PlanarContactInterval
