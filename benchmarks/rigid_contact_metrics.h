#pragma once
#include "physics/physics.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace RigidContactDiagnostic {
using namespace PhysicsEngine;
struct Point {double x,y;};
inline double Cross(Point a,Point b) {return a.x*b.y-a.y*b.x;}
inline Point Difference(Point a,Point b) {return {a.x-b.x,a.y-b.y};}
inline std::vector<Point> Outline(const RigidBody& body) {
    const auto& polygon=dynamic_cast<const Polygon&>(*body.shape);
    const double c=std::cos(body.GetOrientation()),s=std::sin(body.GetOrientation());
    const auto p=body.GetPosition();std::vector<Point> result;
    for (const auto v:polygon.getVertices()) result.push_back({p.x+c*v.x-s*v.y,p.y+s*v.x+c*v.y});
    return result;
}
// Diagnostic geometry uses transformed double coordinates rather than cached
// solver manifolds. SAT measures minimum separating translation for polygons.
inline double PolygonPenetration(const std::vector<Point>& a,const std::vector<Point>& b) {
    double minimum=std::numeric_limits<double>::infinity();
    for (const auto* outline:{&a,&b}) for (std::size_t i=0;i<outline->size();++i) {
        const auto edge=Difference((*outline)[(i+1)%outline->size()],(*outline)[i]);
        const double length=std::hypot(edge.x,edge.y),nx=-edge.y/length,ny=edge.x/length;
        double amin=1e100,amax=-1e100,bmin=1e100,bmax=-1e100;
        for (const auto v:a) {const double t=nx*v.x+ny*v.y;amin=std::min(amin,t);amax=std::max(amax,t);}
        for (const auto v:b) {const double t=nx*v.x+ny*v.y;bmin=std::min(bmin,t);bmax=std::max(bmax,t);}
        if (amax<=bmin||bmax<=amin) return 0;
        minimum=std::min(minimum,std::min(amax-bmin,bmax-amin));
    }
    return minimum;
}
inline double DiskPolygonPenetration(Point center,double radius,const std::vector<Point>& vertices) {
    bool positive=false,negative=false;double closest=1e100;
    for (std::size_t i=0;i<vertices.size();++i) {
        const auto a=vertices[i],edge=Difference(vertices[(i+1)%vertices.size()],a),offset=Difference(center,a);
        const double cross=Cross(edge,offset);positive=positive||cross>0;negative=negative||cross<0;
        const double t=std::clamp((offset.x*edge.x+offset.y*edge.y)/(edge.x*edge.x+edge.y*edge.y),0.0,1.0);
        closest=std::min(closest,std::hypot(offset.x-t*edge.x,offset.y-t*edge.y));
    }
    return positive&&negative?std::max(0.0,radius-closest):radius+closest;
}
inline double Penetration(const RigidBody& a,const RigidBody& b) {
    const bool ac=a.shape->type==ShapeType::CIRCLE,bc=b.shape->type==ShapeType::CIRCLE;
    if (!ac&&!bc) return PolygonPenetration(Outline(a),Outline(b));
    if (ac&&bc) return std::max(0.0,double(a.shape->GetRadius())+b.shape->GetRadius()-
        std::hypot(double(a.position.x)-b.position.x,double(a.position.y)-b.position.y));
    const auto& disk=ac?a:b;const auto& polygon=ac?b:a;
    return DiskPolygonPenetration({disk.position.x,disk.position.y},disk.shape->GetRadius(),Outline(polygon));
}
struct Moments {double px=0,py=0,angular=0,kinetic=0,potential=0;};
inline Moments Measure(const std::vector<RigidBodyPtr>& bodies,double gravity) {
    Moments out;
    for (const auto& body:bodies) {
        const double m=body->GetMass(),i=body->GetInertia(),x=body->position.x,y=body->position.y;
        const double vx=body->velocity.x,vy=body->velocity.y,w=body->angularVelocity;
        if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(vx)||!std::isfinite(vy)||!std::isfinite(w)
            ||!std::isfinite(body->orientation)) throw std::runtime_error("Nonfinite diagnostic body state");
        out.px+=m*vx;out.py+=m*vy;out.angular+=m*(x*vy-y*vx)+i*w;
        out.kinetic+=0.5*(m*(vx*vx+vy*vy)+i*w*w);out.potential+=m*gravity*y;
    }
    if(!std::isfinite(out.px)||!std::isfinite(out.py)||!std::isfinite(out.angular)||!std::isfinite(out.kinetic)||!std::isfinite(out.potential))
        throw std::runtime_error("Nonfinite diagnostic aggregate");
    return out;
}
struct Controls {float dt=1.0f/60;int iterations=10;float warmStart=0.8f;double duration=2;};
inline void Validate(const Controls& c) {
    if (!std::isfinite(c.dt)||c.dt<1.0f/10000||c.dt>0.1f||c.iterations<1||c.iterations>64
        ||!std::isfinite(c.warmStart)||c.warmStart<0||c.warmStart>1
        ||!std::isfinite(c.duration)||c.duration<0.1||c.duration>8)
        throw std::invalid_argument("Diagnostic controls outside bounded dt/iteration/warm/duration ranges");
}
struct FixtureSpec {
    std::string name;int boxes=1;bool incline=false,sliding=false,impact=false,offCenter=false;
    float massRatio=1,restitution=0;
};
inline std::vector<FixtureSpec> Fixtures(bool quick) {
    std::vector<FixtureSpec> out{{"rest"},{"stack3",3},{"stack6",6},
        {"incline_stick",1,true},{"incline_slide",1,true,true}};
    if (!quick) out.push_back({"stack12",12});
    for (float ratio:{1.0f,10.0f,1000.0f}) for (float restitution:{0.0f,0.5f,1.0f}) {
        FixtureSpec s;s.name="head_on_m"+std::to_string(static_cast<int>(ratio))+"_e"+std::to_string(restitution);
        s.impact=true;s.massRatio=ratio;s.restitution=restitution;out.push_back(s);
    }
    FixtureSpec off;off.name="off_center";off.impact=true;off.offCenter=true;off.restitution=0.5f;out.push_back(off);
    return out;
}
struct Tracker:ICollisionListener {
    std::uint64_t begins=0,persists=0,ends=0,frames=0,featureChanges=0,twoPointFrames=0;
    std::map<std::pair<std::uint64_t,std::uint64_t>,std::vector<std::uint32_t>> features;
    void onCollisionBegin(const CollisionEvent&) override {++begins;}
    void onCollisionPersist(const CollisionEvent&) override {++persists;}
    void onCollisionEnd(const CollisionEvent&) override {++ends;}
    void onCollision(const CollisionManifold& m) override {
        ++frames;if(m.contactCount==2)++twoPointFrames;
        std::vector<std::uint32_t> now;for(std::uint8_t i=0;i<m.contactCount;++i)now.push_back(m.contacts[i].featureId);
        std::sort(now.begin(),now.end());const auto a=m.A->GetId(),b=m.B->GetId();
        const std::pair<std::uint64_t,std::uint64_t> key{std::min(a,b),std::max(a,b)};
        auto found=features.find(key);if(found!=features.end()&&found->second!=now)++featureChanges;
        features[key]=std::move(now);
    }
};
struct Result {
    std::string fixture;double dt=0,duration=0,warmStart=0;int iterations=0,steps=0,bodies=0;
    double peakPenetration=0,finalPenetration=0,maxDisplacement=0,maxAngleChange=0;
    double finalSpeed=0,terminalPeakSpeed=0,finalAngularSpeed=0;
    double positionalProjection=0,angularProjection=0,downhillDisplacement=0,downhillSpeed=0;
    Moments initial,final;
    double expectedA=0,expectedB=0,expectedOmegaB=0,expectedEnergy=0,velocityError=0,energyError=0;
    double expectedDownhillDisplacement=0,expectedDownhillSpeed=0;
    std::uint64_t begins=0,persists=0,ends=0,featureChanges=0,twoPointFrames=0;
    std::uint64_t integrated=0,broadCandidates=0,narrowCandidates=0,resolved=0,solverIterations=0,constraints=0;
    std::size_t finalPersistent=0,peakPersistent=0;
    std::vector<std::array<double,6>> states;
    std::vector<double> masses,inertias;
    std::uint64_t experimentalPairVisits=0,experimentalScratchSolves=0;
    double experimentalClosingResidual=0,experimentalTangentialSpeed=0;
    std::map<std::string,std::uint64_t> experimentalEligibility;
};
struct NoStepObservation {
    template<class W> void operator()(const W&,Result&) const {}
};
template<class WorldType=World,class StepObservation=NoStepObservation>
inline Result Run(const FixtureSpec& spec,const Controls& controls,StepObservation observe={}) {
    Validate(controls);SimulationConfig config;
    if(spec.boxes<1||spec.boxes>12||!std::isfinite(spec.massRatio)||spec.massRatio<=0
        ||!std::isfinite(spec.restitution)||spec.restitution<0||spec.restitution>1)
        throw std::invalid_argument("Invalid bounded diagnostic fixture");
    config.solverIterations=controls.iterations;config.warmStartFactor=controls.warmStart;
    config.enableSleeping=false;config.enableLinearVelocityLimit=false;config.enableAngularVelocityLimit=false;
    config.restitutionVelocityThreshold=0; // Explicit analytical restitution at all incident speeds.
    if(spec.impact)config.positionCorrectionFactor=0; // Isolate the instantaneous impulse oracle.
    Tracker tracker;WorldType world(config);world.addCollisionListener(&tracker);
    std::vector<RigidBodyPtr> dynamic;std::vector<Point> initialPositions;std::vector<double> initialAngles;
    const auto add=[&](const Shape& shape,Vector2 p,float mass,const Material& material,bool fixed=false) {
        auto body=std::make_shared<RigidBody>(shape,material,p,fixed);if(!fixed){body->SetMass(mass);dynamic.push_back(body);}
        world.addBody(body);return body;
    };
    double gravity=spec.impact?0:double(9.81f),theta=spec.incline?double(float(0.2617993877991494)):0;
    if(spec.impact) {
        const Material material{1,spec.restitution,0,0};
        if(spec.offCenter) {
            add(Circle(0.25f),{-0.749f,0.3f},1,material)->SetVelocity({3,0});
            add(Polygon::MakeBox(1,1),{},2,material);
        } else {
            add(Circle(0.5f),{-0.4995f,0},1,material)->SetVelocity({2,0});
            add(Circle(0.5f),{0.4995f,0},spec.massRatio,material);
        }
    } else {
        const Material material{1,0,spec.sliding?0.05f:0.8f,spec.sliding?0.03f:0.6f};
        const Vector2 normal{static_cast<float>(-std::sin(theta)),static_cast<float>(std::cos(theta))};
        auto floor=add(Polygon::MakeBox(20,1),normal*(-0.5f),1,material,true);floor->SetOrientation(static_cast<float>(theta));
        for(int i=0;i<spec.boxes;++i){auto box=add(Polygon::MakeBox(1,1),normal*(0.5f+static_cast<float>(i)),1,material);box->SetOrientation(static_cast<float>(theta));}
        world.addUniversalForce(std::make_unique<Gravity>(Vector2{0,-static_cast<float>(gravity)}));
    }
    for(const auto& b:dynamic){initialPositions.push_back({b->position.x,b->position.y});initialAngles.push_back(b->orientation);}
    Result r;r.fixture=spec.name;r.dt=spec.impact?0:controls.dt;r.warmStart=controls.warmStart;r.iterations=controls.iterations;
    r.steps=spec.impact?1:static_cast<int>(std::ceil(controls.duration/controls.dt));r.bodies=static_cast<int>(world.getBodies().size());
    r.duration=r.steps*r.dt;r.initial=Measure(dynamic,gravity);
    if(spec.impact) {
        const double massB=dynamic[1]->GetMass(),u=dynamic[0]->velocity.x,e=spec.restitution;
        const double y=spec.offCenter?dynamic[0]->position.y:0;
        // Uniform box I=m*(width²+height²)/12, circle impact is central.
        const double inertia=spec.offCenter?massB/6:1;
        const double denominator=1+1/massB+(spec.offCenter?y*y/inertia:0);
        const double impulse=(1+e)*u/denominator;
        r.expectedA=u-impulse;r.expectedB=impulse/massB;r.expectedOmegaB=spec.offCenter?-y*impulse/inertia:0;
        r.expectedEnergy=r.initial.kinetic-0.5*(1-e*e)*u*u/denominator;
    }
    for(int step=0;step<r.steps;++step) {
        world.step(static_cast<float>(r.dt));observe(world,r);Measure(dynamic,gravity);
        const auto s=world.getLastStepStatistics();r.integrated+=s.integratedBodyCount;r.broadCandidates+=s.broadPhaseCandidateCount;
        r.narrowCandidates+=s.narrowPhaseCandidateCount;r.resolved+=s.resolvedContactCount;
        r.solverIterations+=s.solverIterationCount;r.constraints+=s.solvedConstraintCount;
        r.peakPersistent=std::max(r.peakPersistent,world.getPersistentContactCount());
        double penetration=0;const auto& all=world.getBodies();
        for(std::size_t i=0;i<all.size();++i)for(std::size_t j=i+1;j<all.size();++j)penetration=std::max(penetration,Penetration(*all[i],*all[j]));
        r.peakPenetration=std::max(r.peakPenetration,penetration);r.finalPenetration=penetration;
        for(const auto& b:dynamic)if(step>=3*r.steps/4)r.terminalPeakSpeed=std::max(r.terminalPeakSpeed,std::hypot(double(b->velocity.x),b->velocity.y));
    }
    r.final=Measure(dynamic,gravity);r.finalPersistent=world.getPersistentContactCount();
    for(std::size_t i=0;i<dynamic.size();++i) {
        const auto& b=dynamic[i];r.finalSpeed=std::max(r.finalSpeed,std::hypot(double(b->velocity.x),b->velocity.y));
        r.finalAngularSpeed=std::max(r.finalAngularSpeed,std::abs(double(b->angularVelocity)));
        const double movement=std::hypot(double(b->position.x)-initialPositions[i].x,double(b->position.y)-initialPositions[i].y);
        const double angle=std::abs(std::remainder(double(b->orientation)-initialAngles[i],2*3.14159265358979323846));
        r.maxDisplacement=std::max(r.maxDisplacement,movement);r.maxAngleChange=std::max(r.maxAngleChange,angle);
        r.states.push_back({b->position.x,b->position.y,b->orientation,b->velocity.x,b->velocity.y,b->angularVelocity});
        r.masses.push_back(b->GetMass());r.inertias.push_back(b->GetInertia());
    }
    if(spec.impact) {
        r.positionalProjection=r.maxDisplacement;r.angularProjection=r.maxAngleChange;
        r.velocityError=std::max({std::abs(double(dynamic[0]->velocity.x)-r.expectedA),
            std::abs(double(dynamic[1]->velocity.x)-r.expectedB),std::abs(double(dynamic[1]->angularVelocity)-r.expectedOmegaB)});
        r.energyError=r.final.kinetic-r.expectedEnergy;
    }
    if(spec.incline) {
        const double tx=-std::cos(theta),ty=-std::sin(theta);
        r.downhillDisplacement=tx*(dynamic[0]->position.x-initialPositions[0].x)+ty*(dynamic[0]->position.y-initialPositions[0].y);
        r.downhillSpeed=tx*dynamic[0]->velocity.x+ty*dynamic[0]->velocity.y;
        const double acceleration=spec.sliding?gravity*(std::sin(theta)-double(0.03f)*std::cos(theta)):0;
        r.expectedDownhillSpeed=acceleration*r.duration;r.expectedDownhillDisplacement=0.5*acceleration*r.duration*r.duration;
    }
    r.begins=tracker.begins;r.persists=tracker.persists;r.ends=tracker.ends;r.featureChanges=tracker.featureChanges;r.twoPointFrames=tracker.twoPointFrames;
    world.removeCollisionListener(&tracker);return r;
}
} // namespace RigidContactDiagnostic
