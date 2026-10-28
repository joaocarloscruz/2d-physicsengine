// Headless rigid-body scenario suite. Each row states its physical expectation
// and pass threshold up front; the process exits nonzero if any gated row fails.
// Run with --verbose for per-row details. All scenes use the default World
// config unless a row says otherwise.
#include "physics/physics.h"
#include "physics/core/collisions/collision_dispatcher.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace PhysicsEngine;

namespace {
constexpr double G = 9.81;
constexpr double Pi = 3.14159265358979323846;
constexpr float Dt = 1.0f / 60.0f;

struct Row {
    std::string name, expectation, threshold;
    double value;
    bool pass;
    bool gated = true;
};
std::vector<Row> rows;

void Check(const std::string& name, const std::string& expectation, const std::string& threshold,
           double value, bool pass, bool gated = true) {
    rows.push_back({name, expectation, threshold, value, pass, gated});
}

Material Mat(float density = 1, float restitution = 0.2f, float mus = 0.6f, float mud = 0.4f) {
    Material m; m.density = density; m.restitution = restitution; m.staticFriction = mus; m.dynamicFriction = mud;
    return m;
}

RigidBodyPtr Add(World& w, const Shape& s, Vector2 p, const Material& m = Mat(), bool fixed = false, float angle = 0) {
    auto b = std::make_shared<RigidBody>(s, m, p, fixed);
    b->SetOrientation(angle);
    w.addBody(b);
    return b;
}

// Ground top at y = 0.
RigidBodyPtr Ground(World& w, const Material& m = Mat(1, 0.2f), float width = 200) {
    auto g = Add(w, Polygon::MakeBox(width, 2), {0, -1}, m, true);
    w.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -float(G)}));
    return g;
}

bool Jointed(World& w, const RigidBodyPtr& a, const RigidBodyPtr& b) {
    for (const auto& j : w.getJoints())
        if (!j->getCollideConnected() && ((j->getBodyA() == a && j->getBodyB() == b) || (j->getBodyA() == b && j->getBodyB() == a)))
            return true;
    return false;
}

double Penetration(World& w) {
    double worst = 0;
    const auto& b = w.getBodies();
    for (size_t i = 0; i < b.size(); ++i) {
        const AABB ai = b[i]->GetAABB();
        for (size_t j = i + 1; j < b.size(); ++j) {
            if (b[i]->IsStatic() && b[j]->IsStatic()) continue;
            if (!b[i]->CanCollideWith(*b[j]) || Jointed(w, b[i], b[j])) continue;
            const AABB aj = b[j]->GetAABB();
            if (ai.max.x < aj.min.x || aj.max.x < ai.min.x || ai.max.y < aj.min.y || aj.max.y < ai.min.y) continue;
            worst = std::max(worst, double(CheckCollision(b[i].get(), b[j].get()).penetration));
        }
    }
    return worst;
}

double Energy(World& w) {
    double e = 0;
    for (const auto& b : w.getBodies()) {
        if (b->IsStatic()) continue;
        e += 0.5 * b->GetMass() * b->velocity.magnitudeSquared()
           + 0.5 * b->GetInertia() * double(b->angularVelocity) * b->angularVelocity
           + b->GetMass() * G * b->position.y;
    }
    return e;
}

double MaxSpeed(World& w) {
    double s = 0;
    for (const auto& b : w.getBodies())
        if (!b->IsStatic()) s = std::max(s, double(b->velocity.magnitude()));
    return s;
}

double JointError(World& w) {
    double e = 0;
    for (const auto& j : w.getJoints()) {
        if (!std::dynamic_pointer_cast<RevoluteJoint>(j)) continue;
        e = std::max(e, double((j->getAnchorA() - j->getAnchorB()).magnitude()));
    }
    return e;
}

std::string Fmt(const char* f, double v) { char s[96]; std::snprintf(s, sizeof s, f, v); return s; }

// Tracks drift/tilt/speed after a settle time, and penetration throughout.
struct Rest { double drift = 0, tilt = 0, speed = 0, peakPen = 0, settledPen = 0; };
Rest RunRest(World& w, double seconds, double settle, float dt = Dt) {
    Rest r; std::vector<Vector2> p0; std::vector<float> a0;
    const int steps = int(seconds / dt + 0.5), settleStep = int(settle / dt + 0.5);
    for (int s = 0; s < steps; ++s) {
        w.step(dt);
        const double pen = Penetration(w);
        r.peakPen = std::max(r.peakPen, pen);
        if (s == settleStep) for (auto& b : w.getBodies()) { p0.push_back(b->position); a0.push_back(b->orientation); }
        if (s > settleStep) {
            r.settledPen = std::max(r.settledPen, pen);
            size_t k = 0;
            for (auto& b : w.getBodies()) {
                if (!b->IsStatic()) {
                    r.drift = std::max(r.drift, double((b->position - p0[k]).magnitude()));
                    r.tilt = std::max(r.tilt, std::abs(std::remainder(double(b->orientation - a0[k]), 2 * Pi)));
                    r.speed = std::max(r.speed, double(b->velocity.magnitude()));
                }
                ++k;
            }
        }
    }
    return r;
}

// ---------------------------------------------------------------- 1. mass ratios
void MassRatios() {
    {   // 100 kg slab on two 1 kg boxes.
        World w; Ground(w);
        Add(w, Polygon::MakeBox(1, 1), {-0.75f, 0.5f});
        Add(w, Polygon::MakeBox(1, 1), {0.75f, 0.5f});
        Add(w, Polygon::MakeBox(3, 0.5f), {0, 1.25f}, Mat(100.0f / 1.5f));
        const Rest r = RunRest(w, 5, 1);
        Check("mass 100kg on 1kg: drift", "static load, no creep", "< 0.01 m", r.drift, r.drift < 0.01);
        Check("mass 100kg on 1kg: penetration", "settles near slop (0.005)", "< 0.02 m", r.settledPen, r.settledPen < 0.02);
    }
    {   // 50 kg box on a 1 kg box.
        World w; Ground(w);
        Add(w, Polygon::MakeBox(1, 1), {0, 0.5f});
        Add(w, Polygon::MakeBox(1, 1), {0, 1.5f}, Mat(50));
        const Rest r = RunRest(w, 5, 1);
        Check("mass 50kg on 1kg: drift", "static load, no creep", "< 0.01 m", r.drift, r.drift < 0.01);
        Check("mass 50kg on 1kg: penetration", "settles near slop", "< 0.02 m", r.settledPen, r.settledPen < 0.02);
    }
    {   // Level 50 kg seesaw on a revolute pivot with a 0.25 kg ball near each end.
        World w; Ground(w);
        auto pivot = Add(w, Circle(0.05f), {0, 1}, Mat(), true);
        pivot->SetCollisionMaskBits(0);
        auto plank = Add(w, Polygon::MakeBox(4, 0.2f), {0, 1}, Mat(50.0f / 0.8f));
        w.addJoint(std::make_shared<RevoluteJoint>(pivot, plank));
        const float r = 0.2f, density = 0.25f / float(Pi * r * r);
        auto left = Add(w, Circle(r), {-1.5f, 1.1f + r}, Mat(density));
        auto right = Add(w, Circle(r), {1.5f, 1.1f + r}, Mat(density));
        double tilt = 0, pen = 0, drift = 0;
        for (int s = 0; s < 300; ++s) {
            w.step(Dt);
            tilt = std::max(tilt, std::abs(double(plank->orientation)));
            pen = std::max(pen, Penetration(w));
            drift = std::max({drift, std::abs(left->position.x + 1.5), std::abs(right->position.x - 1.5)});
        }
        Check("seesaw light balls: plank tilt", "symmetric load stays level", "< 0.01 rad", tilt, tilt < 0.01);
        Check("seesaw light balls: penetration", "balls rest on plank", "< 0.02 m", pen, pen < 0.02);
        Check("seesaw light balls: ball drift", "balls stay put", "< 0.01 m", drift, drift < 0.01);
    }
}

// ---------------------------------------------------------------- 2. dominoes
void Dominoes() {
    World w; Ground(w);
    std::vector<RigidBodyPtr> d;
    for (int i = 0; i < 20; ++i) d.push_back(Add(w, Polygon::MakeBox(0.1f, 1), {i * 0.5f, 0.5f}));
    d[0]->SetAngularVelocity(-1.5f);
    d[0]->SetVelocity({0.75f, 0});
    double speed = 0, pen = 0, lowest = 1;
    for (int s = 0; s < 8 * 60; ++s) {
        w.step(Dt);
        speed = std::max(speed, MaxSpeed(w));
        pen = std::max(pen, Penetration(w));
        for (auto& b : d) lowest = std::min(lowest, double(b->position.y));
    }
    int fallen = 0;
    for (auto& b : d) fallen += std::abs(std::remainder(double(b->orientation), 2 * Pi)) > 0.3;
    const double last = std::abs(std::remainder(double(d.back()->orientation), 2 * Pi));
    Check("dominoes: fallen count", "all 20 topple", "== 20 (|angle| > 0.3)", fallen, fallen == 20);
    Check("dominoes: last lies flat", "last one falls to the ground", "|angle| > 1.4 rad", last, last > 1.4);
    Check("dominoes: max speed", "no explosion (tip speed ~ sqrt(3gh) = 5.4)", "< 7 m/s", speed, speed < 7);
    Check("dominoes: penetration", "no sinking", "< 0.02 m", pen, pen < 0.02);
    Check("dominoes: lowest center", "flat domino center at 0.05", "> 0.035 m", lowest, lowest > 0.035);
}

// ---------------------------------------------------------------- 3. joints + contacts
std::vector<RigidBodyPtr> MakeChain(World& w, int count, float y, float h, float r) {
    std::vector<RigidBodyPtr> links;
    for (int i = 0; i < count; ++i) links.push_back(Add(w, Capsule(h, r), {(i - 0.5f * (count - 1)) * 2 * h, y}));
    for (int i = 0; i + 1 < count; ++i)
        w.addJoint(std::make_shared<RevoluteJoint>(links[i], links[i + 1], Vector2{h, 0}, Vector2{-h, 0}));
    return links;
}

void ChainDrape() {
    // The hanging segments swing freely next to the box walls, so the chain
    // need not come to rest; jitter would show up as energy injected instead.
    World w; Ground(w);
    Add(w, Polygon::MakeBox(1, 1.5f), {0, 0.75f}, Mat(), true);
    auto links = MakeChain(w, 15, 2.0f, 0.15f, 0.05f);
    double mass = 0;
    for (auto& l : links) mass += l->GetMass();
    double err = 0, pen = 0, rise = 0, low = Energy(w);
    for (int s = 0; s < 10 * 60; ++s) {
        w.step(Dt);
        err = std::max(err, JointError(w));
        pen = std::max(pen, Penetration(w));
        const double e = Energy(w);
        low = std::min(low, e);
        rise = std::max(rise, e - low);
    }
    rise /= mass * G;
    Check("chain drape: joint separation", "links stay connected", "< 0.02 m", err, err < 0.02);
    Check("chain drape: penetration", "links rest on box/ground", "< 0.03 m", pen, pen < 0.03);
    Check("chain drape: energy rise", "E never rises (no jitter injection)", "< 1 cm of chain weight", rise, rise < 0.01);
}

void ChainFlat() {
    World w; Ground(w);
    MakeChain(w, 15, 0.3f, 0.15f, 0.05f);
    double err = 0, pen = 0, late = 0;
    for (int s = 0; s < 4 * 60; ++s) {
        w.step(Dt);
        err = std::max(err, JointError(w));
        pen = std::max(pen, Penetration(w));
        if (s >= 3 * 60) late = std::max(late, MaxSpeed(w));
    }
    Check("chain dropped flat: joint separation", "links stay connected", "< 0.02 m", err, err < 0.02);
    Check("chain dropped flat: penetration", "rests on ground", "< 0.02 m", pen, pen < 0.02);
    Check("chain dropped flat: settled speed (3-4 s)", "comes to rest, no jitter", "< 0.01 m/s", late, late < 0.01);
}

void PendulumStack() {
    World w; Ground(w);
    auto pivot = Add(w, Circle(0.05f), {0, 5}, Mat(), true);
    pivot->SetCollisionMaskBits(0);
    auto bob = Add(w, Polygon::MakeBox(0.5f, 0.5f), {-4, 5}, Mat(8));
    w.addJoint(std::make_shared<RevoluteJoint>(pivot, bob, Vector2{}, Vector2{4, 0}));
    for (int i = 0; i < 4; ++i) Add(w, Polygon::MakeBox(0.5f, 0.5f), {0.55f, 0.25f + 0.5f * i});
    const double e0 = Energy(w), vBob = std::sqrt(2 * G * 4);
    double err = 0, pen = 0, eMax = e0, speed = 0;
    for (int s = 0; s < 4 * 60; ++s) {
        w.step(Dt);
        err = std::max(err, JointError(w));
        pen = std::max(pen, Penetration(w));
        eMax = std::max(eMax, Energy(w));
        for (auto& b : w.getBodies()) if (b != bob && !b->IsStatic()) speed = std::max(speed, double(b->velocity.magnitude()));
    }
    Check("pendulum->stack: joint separation", "rod stays connected through impact", "< 0.02 m", err, err < 0.02);
    Check("pendulum->stack: penetration", "impact resolved", "< 0.03 m", pen, pen < 0.03);
    Check("pendulum->stack: energy gain", "E never above initial", "< 1% of E0", (eMax - e0) / std::abs(e0), eMax <= e0 + 0.01 * std::abs(e0));
    Check("pendulum->stack: target speed", "elastic bound 2*v_bob = 17.7", "< 17.7 m/s", speed, speed < 2 * vBob);
}

void Ragdoll() {
    World w; Ground(w);
    const float y = 3; // build height; dropped lying at 1.5 m
    auto torso = Add(w, Polygon::MakeBox(0.3f, 0.6f), {0, y});
    auto head = Add(w, Circle(0.12f), {0, y + 0.45f});
    std::vector<RigidBodyPtr> parts{torso, head};
    const auto joint = [&](RigidBodyPtr a, RigidBodyPtr b, Vector2 at, float lo, float hi) {
        auto j = std::make_shared<RevoluteJoint>(a, b, at - a->position, at - b->position);
        j->setLimits(true, lo, hi);
        w.addJoint(j);
    };
    joint(torso, head, {0, y + 0.3f}, -0.5f, 0.5f);
    for (float side : {-1.0f, 1.0f}) {
        auto upperArm = Add(w, Polygon::MakeBox(0.1f, 0.3f), {side * 0.22f, y + 0.1f});
        auto lowerArm = Add(w, Polygon::MakeBox(0.1f, 0.3f), {side * 0.22f, y - 0.2f});
        auto upperLeg = Add(w, Polygon::MakeBox(0.1f, 0.3f), {side * 0.08f, y - 0.45f});
        auto lowerLeg = Add(w, Polygon::MakeBox(0.1f, 0.3f), {side * 0.08f, y - 0.75f});
        joint(torso, upperArm, {side * 0.22f, y + 0.25f}, -2.5f, 2.5f);
        joint(upperArm, lowerArm, {side * 0.22f, y - 0.05f}, -2.0f, 0.0f);
        joint(torso, upperLeg, {side * 0.08f, y - 0.3f}, -1.5f, 1.5f);
        joint(upperLeg, lowerLeg, {side * 0.08f, y - 0.6f}, 0.0f, 2.0f);
        parts.insert(parts.end(), {upperArm, lowerArm, upperLeg, lowerLeg});
    }
    // Lay the figure down (joint frames were set upright) so that it can
    // only come to rest, not balance on its feet.
    const float lying = -float(Pi) / 2;
    for (auto& part : parts) {
        const Vector2 d = part->position - Vector2{0, y};
        part->SetPosition({d.y, 1.5f - d.x});
        part->SetOrientation(lying);
    }
    torso->SetAngularVelocity(0.5f);
    double err = 0, pen = 0, late = 0;
    for (int s = 0; s < 6 * 60; ++s) {
        w.step(Dt);
        err = std::max(err, JointError(w));
        pen = std::max(pen, Penetration(w));
        if (s >= 5 * 60) late = std::max(late, MaxSpeed(w));
    }
    Check("ragdoll drop: joint separation", "joints hold through impact", "< 0.02 m", err, err < 0.02);
    Check("ragdoll drop: penetration", "parts rest on ground", "< 0.03 m", pen, pen < 0.03);
    Check("ragdoll drop: settled speed (5-6 s)", "settles, no jitter", "< 0.05 m/s", late, late < 0.05);
}

void MotorCar() {
    World w; Ground(w);
    auto chassis = Add(w, Polygon::MakeBox(2, 0.4f), {0, 0.8f});
    const float r = 0.3f, speed = -10; // clockwise wheels drive toward +x
    std::vector<std::shared_ptr<RevoluteJoint>> motors;
    std::vector<RigidBodyPtr> wheels;
    for (float x : {-0.7f, 0.7f}) {
        auto wheel = Add(w, Circle(r), {x, r});
        auto j = std::make_shared<RevoluteJoint>(chassis, wheel, Vector2{x, -0.5f}, Vector2{});
        j->setMotor(true, speed, 50);
        w.addJoint(j);
        wheels.push_back(wheel);
    }
    double err = 0, tilt = 0, vMin = 1e9, vMax = 0, slip = 0;
    for (int s = 0; s < 4 * 60; ++s) {
        w.step(Dt);
        err = std::max(err, JointError(w));
        if (s >= 3 * 60) {
            tilt = std::max(tilt, std::abs(double(chassis->orientation)));
            vMin = std::min(vMin, double(chassis->velocity.x));
            vMax = std::max(vMax, double(chassis->velocity.x));
            for (auto& wh : wheels) slip = std::max(slip, std::abs(double(wh->velocity.x) + double(wh->angularVelocity) * r));
        }
    }
    const double target = -speed * r;
    Check("motor car: cruise speed (3-4 s)", Fmt("rolling: v = |w| r = %.2f m/s", target), "within 3%",
          vMin, std::abs(vMin - target) < 0.03 * target && std::abs(vMax - target) < 0.03 * target);
    Check("motor car: wheel slip", "rolls without slipping", "|v + w r| < 0.05 m/s", slip, slip < 0.05);
    Check("motor car: chassis tilt", "level on flat ground", "< 0.02 rad", tilt, tilt < 0.02);
    Check("motor car: axle separation", "wheels stay on axles", "< 0.01 m", err, err < 0.01);
}

void Elevator() {
    World w; auto ground = Ground(w);
    auto platform = Add(w, Polygon::MakeBox(2, 0.2f), {0, 0.1f}, Mat(5));
    auto slider = std::make_shared<PrismaticJoint>(ground, platform, Vector2{0, 1}, Vector2{0, 1.0f}, Vector2{});
    slider->setMotor(true, 1.0f, 2000);
    w.addJoint(slider);
    auto box = Add(w, Polygon::MakeBox(1, 1), {0.3f, 0.7f});
    double lag = 0, pen = 0, slide = 0, platformSpeed = 0;
    for (int s = 0; s < 2 * 60; ++s) {
        w.step(Dt);
        pen = std::max(pen, Penetration(w));
        if (s >= 60) {
            lag = std::max(lag, std::abs(double(box->velocity.y - platform->velocity.y)));
            platformSpeed = std::max(platformSpeed, std::abs(double(platform->velocity.y) - 1.0));
            slide = std::max(slide, std::abs(double(box->position.x) - 0.3));
        }
    }
    Check("prismatic elevator: platform speed", "motor holds 1 m/s under load", "|v - 1| < 0.02 m/s", platformSpeed, platformSpeed < 0.02);
    Check("prismatic elevator: rider follows", "box rides with platform (1-2 s)", "|dv| < 0.02 m/s", lag, lag < 0.02);
    Check("prismatic elevator: penetration", "box rests on platform", "< 0.01 m", pen, pen < 0.01);
    Check("prismatic elevator: rider slide", "no sideways creep", "< 0.005 m", slide, slide < 0.005);
}

void TallStack(int count, bool misaligned, bool gated, float hz = 60) {
    // Misaligned: hand-placed boxes with offsets of +-1 cm and 0.005 rad.
    World w; Ground(w);
    for (int i = 0; i < count; ++i)
        Add(w, Polygon::MakeBox(1, 1), {misaligned ? 0.01f * float((i * 7) % 3 - 1) : 0.0f,
            0.5f + (misaligned ? 1.001f : 1.0f) * i}, Mat(), false, misaligned ? 0.005f * float((i * 5) % 3 - 1) : 0.0f);
    const Rest r = RunRest(w, 30, 2, 1.0f / hz);
    std::string tag = "stack of " + std::to_string(count) + (misaligned ? " misaligned" : " aligned") + " (30 s"
        + (hz != 60 ? Fmt(", %.0f Hz)", hz) : ")");
    if (!gated) tag += " [known limit]";
    Check(tag + ": drift", "static (a 1 m-wide tower stands)", "< 0.02 m", r.drift, r.drift < 0.02, gated);
    Check(tag + ": tilt", "upright", "< 0.01 rad", r.tilt, r.tilt < 0.01, gated);
}

// ---------------------------------------------------------------- 4. fast objects
void FastBox(float speed, float spin) {
    World w;
    const float wallX = 6;
    Add(w, Polygon::MakeBox(0.1f, 6), {wallX, 0}, Mat(), true);
    auto box = Add(w, Polygon::MakeBox(1, 1), {0, 0});
    box->SetVelocity({speed, 0});
    box->SetAngularVelocity(spin);
    double pen = 0, maxX = -1e9;
    for (int s = 0; s < 60; ++s) {
        w.step(Dt);
        pen = std::max(pen, Penetration(w));
        maxX = std::max(maxX, double(box->position.x));
    }
    const std::string tag = Fmt("fast box %.0f m/s", speed) + (spin != 0 ? Fmt(" spin %.0f", spin) : "");
    Check(tag + ": stays in front", "never passes 0.1 m wall (center < wall face)", "center x < 5.95", maxX, maxX < wallX - 0.05);
    Check(tag + ": penetration", "speculative contact stops it at the face", "< 0.05 m", pen, pen < 0.05);
}

void FastCircleStack() {
    World w; Ground(w);
    for (int i = 0; i < 5; ++i) Add(w, Polygon::MakeBox(1, 1), {5, 0.5f + i});
    auto ball = Add(w, Circle(0.2f), {0, 1.5f}, Mat(5));
    ball->SetVelocity({60, 0});
    const double e0 = Energy(w);
    double pen = 0, eMax = e0;
    for (int s = 0; s < 90; ++s) {
        w.step(Dt);
        pen = std::max(pen, Penetration(w));
        eMax = std::max(eMax, Energy(w));
    }
    const bool passed = ball->position.x > 5.5f && ball->velocity.x > 0 && std::abs(ball->position.y - 1.5f) < 0.3f;
    Check("fast circle 60 m/s -> stack: penetration", "no tunneling into boxes", "< 0.05 m", pen, pen < 0.05);
    Check("fast circle 60 m/s -> stack: energy", "no energy gain", "< 1% of E0", (eMax - e0) / e0, eMax <= 1.01 * e0);
    Check("fast circle 60 m/s -> stack: not through", "ball does not pass straight through", "0 = stopped", passed, !passed);
}

// ---------------------------------------------------------------- 5. friction
struct Ramp { RigidBodyPtr body; Vector2 down, normal; };
Ramp MakeRamp(World& w, double theta, const Material& m) {
    w.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -float(G)}));
    Ramp r;
    r.body = Add(w, Polygon::MakeBox(40, 1), {0, 0}, m, true, float(theta));
    r.down = {-float(std::cos(theta)), -float(std::sin(theta))};
    r.normal = {-float(std::sin(theta)), float(std::cos(theta))};
    return r;
}
// Place a body on the ramp, `offset` above the surface along the normal.
Vector2 OnRamp(const Ramp& r, float along, float offset) {
    return r.down * -along + r.normal * (0.5f + offset);
}

void RampFriction(double degrees, float mus, float mud) {
    const double theta = degrees * Pi / 180;
    World w;
    const Material m = Mat(1, 0, mus, mud);
    Ramp ramp = MakeRamp(w, theta, m);
    auto box = Add(w, Polygon::MakeBox(1, 0.5f), OnRamp(ramp, 0, 0.25f), m, false, float(theta));
    const Vector2 p0 = box->position;
    const double t = 1.5;
    for (int s = 0; s < int(t * 60); ++s) w.step(Dt);
    const double travel = (box->position - p0).dot(ramp.down);
    const double mu = std::sqrt(double(mus) * mus);
    const std::string tag = Fmt("ramp %.0f deg box", degrees);
    if (std::tan(theta) < mu) {
        Check(tag + ": sticks", Fmt("tan < mu_s=%.2f: static", mu), "travel < 0.005 m", travel, std::abs(travel) < 0.005);
    } else {
        const double a = G * (std::sin(theta) - std::sqrt(double(mud) * mud) * std::cos(theta));
        const double measured = 2 * travel / (t * t);
        Check(tag + ": slide accel", Fmt("a = g(sin - mu_d cos) = %.3f", a), "within 3%", measured,
              std::abs(measured - a) < 0.03 * a);
    }
}

void SlideToRest() {
    World w; const Material m = Mat(1, 0, 0.6f, 0.4f); Ground(w, m);
    auto box = Add(w, Polygon::MakeBox(1, 0.5f), {0, 0.25f}, m);
    const double v0 = 5;
    box->SetVelocity({float(v0), 0});
    for (int s = 0; s < 3 * 60; ++s) w.step(Dt);
    const double expected = v0 * v0 / (2 * 0.4 * G), d = box->position.x;
    Check("slide to rest 5 m/s: distance", Fmt("v^2/(2 mu_d g) = %.3f m", expected), "within 3%", d,
          std::abs(d - expected) < 0.03 * expected);
    Check("slide to rest 5 m/s: stopped", "at rest after 3 s", "|v| < 0.01 m/s", box->velocity.magnitude(),
          box->velocity.magnitude() < 0.01);
}

// ---------------------------------------------------------------- 6. restitution
void Bounce(float e) {
    World w; Ground(w, Mat(1, e));
    const float r = 0.25f;
    auto ball = Add(w, Circle(r), {0, 1 + r}, Mat(1, e));
    // Apex of each rebound (height of the ball's bottom above the ground).
    std::vector<double> apex; bool rising = false; double top = 0;
    for (int s = 0; s < 6 * 60; ++s) {
        w.step(Dt);
        const double vy = ball->velocity.y, hgt = ball->position.y - r;
        if (vy > 0.05) { rising = true; top = std::max(top, hgt); }
        else if (rising && vy < 0) { apex.push_back(top); rising = false; top = 0; }
    }
    const double first = apex.empty() ? 0 : apex[0];
    const double target = e * e;
    Check(Fmt("bounce e=%.1f: first rebound", e), Fmt("e^2 * 1 m = %.3f m", target), "within 8%", first,
          std::abs(first - target) < 0.08 * target);
    if (e == 1.0f) {
        const double peak = apex.empty() ? 0 : *std::max_element(apex.begin(), apex.end());
        Check("bounce e=1.0: no energy gain", "rebounds never above drop height", "max apex < 1.02 m", peak, peak < 1.02);
        const double third = apex.size() >= 3 ? apex[2] : 0;
        Check("bounce e=1.0: keeps bouncing", "no sticking", "3rd apex > 0.9 m", third, third > 0.9);
    }
}

void BouncyStack() {
    World w; Ground(w, Mat(1, 0.8f));
    const float r = 0.25f;
    for (int i = 0; i < 3; ++i) Add(w, Circle(r), {0, 1 + r + 2 * r * i + 0.001f * i}, Mat(1, 0.8f));
    const double e0 = Energy(w);
    double eMax = e0, pen = 0;
    for (int s = 0; s < 5 * 60; ++s) {
        w.step(Dt);
        eMax = std::max(eMax, Energy(w));
        pen = std::max(pen, Penetration(w));
    }
    Check("bouncy ball stack e=0.8: energy", "no energy gain", "< 1% of E0", (eMax - e0) / e0, eMax <= 1.01 * e0);
    Check("bouncy ball stack e=0.8: penetration", "no sinking", "< 0.03 m", pen, pen < 0.03);
}

// ---------------------------------------------------------------- 7. rolling
void RollingDisk(double degrees, float mu) {
    const double theta = degrees * Pi / 180;
    World w; const Material m = Mat(1, 0, mu, mu);
    Ramp ramp = MakeRamp(w, theta, m);
    const float r = 0.5f;
    auto disk = Add(w, Circle(r), OnRamp(ramp, 0, r), m);
    const Vector2 p0 = disk->position;
    const double t = 1.5;
    for (int s = 0; s < int(t * 60); ++s) w.step(Dt);
    const double travel = (disk->position - p0).dot(ramp.down), measured = 2 * travel / (t * t);
    const double speed = disk->velocity.dot(ramp.down), spin = double(disk->angularVelocity) * r; // rolling toward -x is CCW
    const std::string tag = Fmt("disk %.0f deg", degrees) + Fmt(" mu=%.1f", mu);
    if (std::tan(theta) <= 3 * mu) {
        const double a = 2.0 / 3.0 * G * std::sin(theta);
        Check(tag + ": rolling accel", Fmt("a = 2/3 g sin = %.3f", a), "within 3%", measured, std::abs(measured - a) < 0.03 * a);
        Check(tag + ": no slip", "v = omega r", "|v - wr| < 2% v", std::abs(speed - spin) / speed, std::abs(speed - spin) < 0.02 * speed);
    } else {
        const double a = G * (std::sin(theta) - mu * std::cos(theta));
        Check(tag + ": slipping accel", Fmt("a = g(sin - mu cos) = %.3f", a), "within 3%", measured, std::abs(measured - a) < 0.03 * a);
    }
}

void CapsuleRamp(double degrees) {
    const double theta = degrees * Pi / 180;
    World w; const Material m = Mat(1, 0, 0.6f, 0.4f);
    Ramp ramp = MakeRamp(w, theta, m);
    const float r = 0.2f;
    auto cap = Add(w, Capsule(0.4f, r), OnRamp(ramp, 0, r), m, false, float(theta));
    const Vector2 p0 = cap->position;
    const double t = 1.5;
    for (int s = 0; s < int(t * 60); ++s) w.step(Dt);
    const double travel = (cap->position - p0).dot(ramp.down);
    const std::string tag = Fmt("capsule lying on %.0f deg ramp", degrees);
    if (std::tan(theta) < 0.6) {
        Check(tag + ": sticks", "tan < mu_s: static", "travel < 0.005 m", travel, std::abs(travel) < 0.005);
    } else {
        const double a = G * (std::sin(theta) - 0.4 * std::cos(theta)), measured = 2 * travel / (t * t);
        Check(tag + ": slide accel", Fmt("a = g(sin - mu_d cos) = %.3f", a), "within 3%", measured, std::abs(measured - a) < 0.03 * a);
    }
}

void CapsuleRolling() {
    // In 2D only a zero half-length capsule can roll; it must match the disk.
    const double theta = 20 * Pi / 180;
    World w; const Material m = Mat(1, 0, 0.6f, 0.4f);
    Ramp ramp = MakeRamp(w, theta, m);
    auto cap = Add(w, Capsule(0.0f, 0.3f), OnRamp(ramp, 0, 0.3f), m);
    const Vector2 p0 = cap->position;
    double pen = 0;
    for (int s = 0; s < 90; ++s) { w.step(Dt); pen = std::max(pen, Penetration(w)); }
    const double travel = (cap->position - p0).dot(ramp.down), measured = 2 * travel / (1.5 * 1.5);
    const double a = 2.0 / 3.0 * G * std::sin(theta);
    Check("capsule h=0 rolling 20 deg: accel", Fmt("disk: 2/3 g sin = %.3f", a), "within 3%", measured, std::abs(measured - a) < 0.03 * a);
    Check("capsule h=0 rolling 20 deg: penetration", "stays on surface", "< 0.01 m", pen, pen < 0.01);
}

// ---------------------------------------------------------------- 8. sleeping
void Sleeping() {
    SimulationConfig config; config.enableSleeping = true;
    {
        World w(config); Ground(w);
        std::vector<RigidBodyPtr> pile;
        for (int c = 0; c < 3; ++c) for (int i = 0; i < 3; ++i)
            pile.push_back(Add(w, Polygon::MakeBox(1, 1), {c * 1.2f, 0.52f + 1.02f * i}));
        int asleepAt = -1;
        for (int s = 0; s < 6 * 60 && asleepAt < 0; ++s) {
            w.step(Dt);
            if (std::none_of(pile.begin(), pile.end(), [](const RigidBodyPtr& b) { return b->IsAwake(); })) asleepAt = s;
        }
        const double t = asleepAt < 0 ? 99 : (asleepAt + 1) * Dt;
        Check("sleep: pile of 9 falls asleep", "rests, then sleeps", "within 3 s", t, t <= 3);
        // Hit the middle of column 0 with a ball.
        auto ball = Add(w, Circle(0.2f), {-3, 1.55f}, Mat(2));
        ball->SetVelocity({10, 0});
        const Vector2 hit0 = pile[1]->position;
        double pen = 0; int wokeAt = -1;
        for (int s = 0; s < 60; ++s) {
            w.step(Dt);
            pen = std::max(pen, Penetration(w));
            if (wokeAt < 0 && pile[1]->IsAwake()) wokeAt = s;
        }
        const double moved = (pile[1]->position - hit0).magnitude();
        Check("sleep: ball wakes pile", "struck box wakes and moves", "moved > 0.05 m", moved, moved > 0.05);
        Check("sleep: impact on sleeping pile", "no tunneling into sleeping box", "pen < 0.03 m", pen, pen < 0.03);
        // Let everything sleep again, then remove a bottom support (column 2).
        for (int s = 0; s < 6 * 60; ++s) w.step(Dt);
        const Vector2 above0 = pile[7]->position;
        w.removeBody(pile[6]);
        for (int s = 0; s < 60; ++s) w.step(Dt);
        const double drop = above0.y - pile[7]->position.y;
        Check("sleep: remove dynamic support", "box above wakes and falls ~1 m", "drop > 0.8 m", drop, drop > 0.8);
    }
    {
        World w(config); Ground(w);
        auto shelf = Add(w, Polygon::MakeBox(2, 0.2f), {0, 3}, Mat(), true);
        auto box = Add(w, Polygon::MakeBox(1, 1), {0, 3.6f});
        for (int s = 0; s < 3 * 60; ++s) w.step(Dt);
        const bool slept = !box->IsAwake();
        w.removeBody(shelf);
        const float y0 = box->position.y;
        for (int s = 0; s < 30; ++s) w.step(Dt);
        const double drop = y0 - box->position.y;
        Check("sleep: box sleeps on static shelf", "sleeps", "asleep after 3 s", slept, slept);
        Check("sleep: remove static support", "free fall 0.5 g t^2 = 1.23 m in 0.5 s", "drop > 1.1 m", drop, drop > 1.1);
    }
}

// ---------------------------------------------------------------- 9. scale
void Scale(float size, bool scaledConfig) {
    SimulationConfig config;
    if (scaledConfig) {
        // Length tolerances are absolute; scale them with the scene.
        config.penetrationSlop *= size;
        config.maxPositionCorrection *= size;
    }
    World w(config); Ground(w, Mat(), 200 * std::max(size, 1.0f));
    for (int i = 0; i < 5; ++i) Add(w, Polygon::MakeBox(size, size), {0, size * (0.5f + i)});
    const Rest r = RunRest(w, 6, 2);
    const std::string tag = Fmt("stack of 5, %.2f m boxes", size) + (scaledConfig ? " (scaled slop)" : " (default config)");
    Check(tag + ": drift", "static", "< 1% of size", r.drift / size, r.drift < 0.01 * size, scaledConfig);
    Check(tag + ": penetration", "settles near slop", "< 5% of size", r.settledPen / size, r.settledPen < 0.05 * size, scaledConfig);
    Check(tag + ": tilt", "upright", "< 0.01 rad", r.tilt, r.tilt < 0.01, scaledConfig);
}
} // namespace

int main(int argc, char** argv) {
    const bool verbose = argc > 1 && std::strcmp(argv[1], "--verbose") == 0;
    const auto t0 = std::chrono::steady_clock::now();
    MassRatios();
    Dominoes();
    ChainDrape();
    ChainFlat();
    PendulumStack();
    Ragdoll();
    MotorCar();
    Elevator();
    TallStack(20, false, true);
    TallStack(12, true, true);
    TallStack(20, true, true, 120);
    TallStack(15, true, true);
    TallStack(20, true, true);
    FastBox(50, 0); FastBox(100, 0); FastBox(50, 25);
    FastCircleStack();
    RampFriction(25, 0.6f, 0.4f); RampFriction(40, 0.6f, 0.4f);
    SlideToRest();
    Bounce(0.5f); Bounce(1.0f);
    BouncyStack();
    RollingDisk(20, 0.6f); RollingDisk(40, 0.1f);
    CapsuleRamp(25); CapsuleRamp(40);
    CapsuleRolling();
    Sleeping();
    Scale(0.05f, false); Scale(0.05f, true); Scale(20, false); Scale(20, true);
    Scale(0.01f, false); Scale(0.01f, true); Scale(100, false); Scale(100, true);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    int failed = 0;
    for (const auto& r : rows) {
        const char* status = r.pass ? "pass" : r.gated ? "FAIL" : "info";
        failed += !r.pass && r.gated;
        if (verbose)
            std::printf("%-4s | %-52s | %-48s | %-24s | %.5g\n", status, r.name.c_str(), r.expectation.c_str(), r.threshold.c_str(), r.value);
        else
            std::printf("%-4s %-52s %.5g  (%s)\n", status, r.name.c_str(), r.value, r.threshold.c_str());
    }
    std::printf("%zu rows, %d failed, %.0f ms\n", rows.size(), failed, ms);
    return failed == 0 ? 0 : 1;
}
