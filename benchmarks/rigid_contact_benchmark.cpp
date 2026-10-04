#include "rigid_contact_metrics.h"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>

using namespace RigidContactDiagnostic;
namespace {
double Number(const std::string& value) {
    std::size_t count=0;const double out=std::stod(value,&count);
    if(count!=value.size()||!std::isfinite(out))throw std::invalid_argument("Invalid numeric option");
    return out;
}
std::string Escape(const std::string& input) {
    constexpr char hex[]="0123456789abcdef";
    std::string out;for(char c:input){if(c=='"'||c=='\\'){out+='\\';out+=c;}
        else if(static_cast<unsigned char>(c)<0x20){const auto value=static_cast<unsigned char>(c);out+="\\u00";out+=hex[value>>4];out+=hex[value&15];}
        else out+=c;}return out;
}
void Write(std::ostream& out,const Result& r,double milliseconds,const FixtureSpec& spec) {
    const bool impact=spec.impact;
    out<<"{\"fixture\":\""<<r.fixture<<"\",\"classification\":\"observation\",\"dt\":"<<r.dt
       <<",\"duration\":"<<r.duration<<",\"steps\":"<<r.steps<<",\"bodies\":"<<r.bodies
       <<",\"iterations\":"<<r.iterations<<",\"warmStart\":"<<r.warmStart
       <<",\"gravityApplied\":"<<(impact?0:double(9.81f))<<",\"restitution\":"<<spec.restitution
       <<",\"staticFriction\":"<<(impact?0:spec.sliding?double(0.05f):double(0.8f))
       <<",\"dynamicFriction\":"<<(impact?0:spec.sliding?double(0.03f):double(0.6f))
       <<",\"inclineRadians\":"<<(spec.incline?double(float(0.2617993877991494)):0)
       <<",\"oracle\":\""<<(impact?"isolated_instantaneous_impulse":spec.sliding?"ideal_coulomb_slide":"static_equilibrium")<<'"'
       <<",\"positionCorrectionFactor\":"<<(impact?0:0.8)<<",\"penetrationSlop\":0.005"
       <<",\"peakPenetration\":"<<r.peakPenetration<<",\"finalPenetration\":"<<r.finalPenetration
       <<",\"maxDisplacement\":"<<r.maxDisplacement<<",\"maxAngleChange\":"<<r.maxAngleChange
       <<",\"finalMaximumSpeed\":"<<r.finalSpeed<<",\"terminalPeakSpeed\":"<<r.terminalPeakSpeed
       <<",\"finalMaximumAngularSpeed\":"<<r.finalAngularSpeed
       <<",\"positionProjection\":"<<r.positionalProjection<<",\"angularProjection\":"<<r.angularProjection
       <<",\"initialMomentum\":["<<r.initial.px<<','<<r.initial.py<<"],\"finalMomentum\":["<<r.final.px<<','<<r.final.py<<']'
       <<",\"initialAngularMomentum\":"<<r.initial.angular<<",\"finalAngularMomentum\":"<<r.final.angular
       <<",\"initialKineticEnergy\":"<<r.initial.kinetic<<",\"finalKineticEnergy\":"<<r.final.kinetic
       <<",\"initialPotentialEnergy\":"<<r.initial.potential<<",\"finalPotentialEnergy\":"<<r.final.potential
       <<",\"expectedVelocityA\":";if(impact)out<<r.expectedA;else out<<"null";
    out<<",\"expectedVelocityB\":";if(impact)out<<r.expectedB;else out<<"null";
    out<<",\"expectedOmegaB\":";if(impact)out<<r.expectedOmegaB;else out<<"null";
    out<<",\"expectedKineticEnergy\":";if(impact)out<<r.expectedEnergy;else out<<"null";
    out<<",\"impactVelocityError\":";if(impact)out<<r.velocityError;else out<<"null";
    out<<",\"impactEnergyError\":";if(impact)out<<r.energyError;else out<<"null";
    out
       <<",\"downhillDisplacement\":"<<r.downhillDisplacement<<",\"downhillSpeed\":"<<r.downhillSpeed
       <<",\"expectedDownhillDisplacement\":"<<r.expectedDownhillDisplacement
       <<",\"expectedDownhillSpeed\":"<<r.expectedDownhillSpeed
       <<",\"contactBegins\":"<<r.begins<<",\"contactPersists\":"<<r.persists<<",\"contactEnds\":"<<r.ends
       <<",\"featureChanges\":"<<r.featureChanges<<",\"twoPointContactFrames\":"<<r.twoPointFrames
       <<",\"finalPersistentContacts\":"<<r.finalPersistent<<",\"peakPersistentContacts\":"<<r.peakPersistent
       <<",\"integratedBodies\":"<<r.integrated<<",\"broadCandidates\":"<<r.broadCandidates
       <<",\"narrowCandidates\":"<<r.narrowCandidates<<",\"resolvedContacts\":"<<r.resolved
       <<",\"solverIterationVisits\":"<<r.solverIterations<<",\"solvedConstraints\":"<<r.constraints
       <<",\"elapsedMilliseconds\":"<<milliseconds<<",\"finalStates\":[";
    bool first=true;for(const auto& state:r.states){if(!first)out<<',';first=false;out<<'[';
        for(std::size_t i=0;i<state.size();++i){if(i)out<<',';out<<state[i];}out<<']';}out<<"],\"dynamicMasses\":[";
    for(std::size_t i=0;i<r.masses.size();++i){if(i)out<<',';out<<r.masses[i];}out<<"],\"dynamicInertias\":[";
    for(std::size_t i=0;i<r.inertias.size();++i){if(i)out<<',';out<<r.inertias[i];}out<<"]}";
}
}
int main(int argc,char** argv) {
    try {
        bool quick=false;std::string path;std::optional<double> dt,iterations,warm,duration;
        for(int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            if(arg=="--quick"){quick=true;continue;}
            if(arg=="--help"){std::cout<<"rigid_contact_benchmark [--quick] [--output path] [--dt seconds] [--iterations 1..64] [--warm-start 0..1] [--duration .1..8]\n";return 0;}
            if(i+1>=argc)throw std::invalid_argument("Missing option value");
            const std::string value=argv[++i];
            if(arg=="--output")path=value;else if(arg=="--dt")dt=Number(value);
            else if(arg=="--iterations")iterations=Number(value);else if(arg=="--warm-start")warm=Number(value);
            else if(arg=="--duration")duration=Number(value);else throw std::invalid_argument("Unknown option: "+arg);
        }
        if(iterations&&(*iterations!=std::floor(*iterations)||*iterations<1||*iterations>64))throw std::invalid_argument("Iterations must be an integer in 1..64");
        const std::vector<float> dts=dt?std::vector<float>{static_cast<float>(*dt)}:
            quick?std::vector<float>{1.0f/60,1.0f/120}:std::vector<float>{1.0f/60,1.0f/120,1.0f/240};
        const std::vector<int> counts=iterations?std::vector<int>{static_cast<int>(*iterations)}:
            quick?std::vector<int>{4,10}:std::vector<int>{4,10,20};
        const std::vector<float> warms=warm?std::vector<float>{static_cast<float>(*warm)}:
            quick?std::vector<float>{0,0.8f}:std::vector<float>{0,0.8f,1};
        const auto fixtures=Fixtures(quick);std::uint64_t planned=0;
        for(float h:dts)for(int count:counts)for(float factor:warms) {
            const Controls c{h,count,factor,duration.value_or(2)};Validate(c);
            for(const auto& fixture:fixtures)planned+=fixture.impact?1:static_cast<std::uint64_t>(std::ceil(c.duration/h));
        }
        if(planned>250000)throw std::invalid_argument("Matrix exceeds the 250000 total World-step work budget");
        std::ofstream file;if(!path.empty()){file.open(path);if(!file)throw std::runtime_error("Cannot open diagnostic output");}
        std::ostream& out=path.empty()?std::cout:file;
        out<<std::setprecision(17)<<"{\"schemaVersion\":1,\"quick\":"<<(quick?"true":"false")
           <<",\"gravity\":9.81,\"sleeping\":false,\"linearVelocityLimit\":false,\"angularVelocityLimit\":false,\"ccd\":false"
           <<",\"restitutionVelocityThreshold\":0,\"plannedWorldSteps\":"<<planned
           <<",\"contactPersistenceIsCacheHitCount\":false,\"timingIsDeterministic\":false,\"rows\":[";
        bool first=true;int failures=0;
        for(float h:dts)for(int count:counts)for(float factor:warms)for(const auto& fixture:fixtures) {
            const auto start=std::chrono::steady_clock::now();
            if(!first)out<<',';first=false;
            try {
                const auto r=Run(fixture,{h,count,factor,duration.value_or(2)});
                const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
                Write(out,r,elapsed,fixture);
            }catch(const std::exception& error){++failures;out<<"{\"fixture\":\""<<fixture.name<<"\",\"dt\":"<<h
                <<",\"iterations\":"<<count<<",\"warmStart\":"<<factor<<",\"requestedDuration\":"<<duration.value_or(2)
                <<",\"classification\":\"execution_failure\",\"error\":\""<<Escape(error.what())<<"\"}";}
        }
        out<<"],\"executionFailures\":"<<failures<<"}\n";out.flush();if(!out)throw std::runtime_error("Failed writing diagnostic output");return failures?1:0;
    }catch(const std::exception& error){std::cerr<<"rigid_contact_benchmark: "<<error.what()<<'\n';return 1;}
}
