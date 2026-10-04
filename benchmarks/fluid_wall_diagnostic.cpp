#include "fluid_wall_audit.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
using namespace PhysicsEngine;
using namespace FluidWallAudit;
namespace {
void WriteVector(std::ostream& out,D2 v) { out << '[' << v.x << ',' << v.y << ']'; }
void WriteWork(std::ostream& out,const WorkMetrics& m,const std::vector<FluidParticle>& p) {
    double bulkRate=0;std::size_t bulk=0;
    for(std::size_t i=0;i<p.size();++i) if(p[i].position.x>-1+p[i].smoothingLength &&
        p[i].position.x<1-p[i].smoothingLength && p[i].position.y>p[i].smoothingLength &&
        p[i].position.y<1.5f-p[i].smoothingLength) { bulkRate+=m.densityRates[i]/p[i].density;++bulk; }
    out << "{\"pairMechanicalWork\":" << m.pairMechanicalWork << ",\"pairInternalEnergyRate\":" << m.pairInternalEnergyRate
        << ",\"pairWorkResidual\":" << m.pairWorkResidual << ",\"wallFluidWork\":" << m.wallFluidWork
        << ",\"inferredReactionWork\":" << m.inferredReactionWork << ",\"wallRelativeMechanicalWork\":" << m.wallRelativeMechanicalWork
        << ",\"wallInternalEnergyRate\":" << m.wallInternalEnergyRate << ",\"wallWorkResidual\":" << m.wallWorkResidual
        << ",\"gravityWork\":" << m.gravityWork << ",\"maximumAbsoluteRateOverRestDensity\":" << m.maximumAbsoluteDensityRate
        << ",\"meanRateOverRestDensity\":" << m.meanDensityRate << ",\"bulkMeanRateOverCurrentDensity\":" << (bulk?bulkRate/bulk:0)
        << ",\"bulkRateSampleCount\":" << bulk << '}';
}
void WriteScene(std::ostream& out,const char* geometry,float dx,float h,SphKernelFamily family,bool initialized,const char* path) {
    const int columns=static_cast<int>(std::lround(2/dx))-1,rows=static_cast<int>(std::lround(1.5f/dx));
    FluidParticleProperties properties; properties.restDensity=1000;properties.mass=1000*dx*dx;properties.smoothingLength=h;properties.viscosity=0;
    std::vector<FluidParticle> particles;
    for(int y=0;y<rows;++y) for(int x=0;x<columns;++x) {
        const Vector2 position{-1+dx+x*dx,dx+y*dx};
        particles.emplace_back(position,Vector2{},properties);
        if(initialized) particles.back().density=static_cast<float>(1000*HydrostaticDensityRatio(1.5-position.y));
    }
    std::vector<FluidBoundaryParticle> walls;
    if(std::string(geometry)=="column") {
        FluidConvexPolygonContainer tank({{-1,0},{1,0},{1,2},{-1,2}});
        FluidBoundarySamplingSettings sampling;sampling.spacing=dx;sampling.supportRadius=h;
        walls=SampleFluidContainerBoundary(tank,sampling);
    } else {
        const int layers=static_cast<int>(std::ceil(h/dx)),edgeSamples=static_cast<int>(std::lround(2/dx));
        for(int layer=0;layer<layers;++layer) for(int i=0;i<edgeSamples;++i) {
            walls.push_back({{-1+(i+0.5f)*dx,-layer*dx},{},dx*dx,{},1});
            if(std::string(geometry)=="corner") walls.push_back({{-1-layer*dx,(i+0.5f)*dx},{},dx*dx,{},1});
        }
    }
    WcsphConfig config;config.speedOfSound=40;config.equationOfStateExponent=7;config.kernelFamily=family;config.externalAcceleration={0,-9.81f};
    config.densityMode=initialized?WcsphDensityMode::Continuity:WcsphDensityMode::Summation;config.densityDiffusion=0;
    WcsphSolver solver(h,config);solver.prepare(particles,walls);
    const auto op=Build(particles,walls,config);
    D2 pair,wall,gravity,actual,localAdjointWall,signedWall;double pressureError=0,accel2=0,bulkAccel2=0,maxReconstruction=0,scaleForce=0;
    std::size_t bulk=0;double bottomAccel2=0,cornerAccel2=0,signedAccel2=0;std::size_t bottom=0,corner=0;
    for(std::size_t i=0;i<particles.size();++i) {
        const auto& p=particles[i];signedWall=signedWall+op.signedExtrapolationWallForces[i];localAdjointWall=localAdjointWall+op.localAdjointWallForces[i];pair=pair+op.fluidForces[i];wall=wall+op.wallForces[i];gravity=gravity+op.gravityForces[i];actual=actual+ToDouble(p.force);
        const D2 reconstructed=op.fluidForces[i]+op.wallForces[i]+op.gravityForces[i];
        maxReconstruction=std::max(maxReconstruction,(ToDouble(p.force)-reconstructed).norm());
        scaleForce=std::max(scaleForce,reconstructed.norm());
        const double expected=HydrostaticPressure(1.5-p.position.y);
        const double error=(p.pressure-expected)/(1000*9.81*1.5);pressureError+=error*error;
        const double acceleration=ToDouble(p.force).norm()/p.mass;accel2+=acceleration*acceleration;
        const double signedAcceleration=(op.fluidForces[i]+op.gravityForces[i]+op.signedExtrapolationWallForces[i]).norm()/p.mass;
        signedAccel2+=signedAcceleration*signedAcceleration;
        if(p.position.x>-1+h && p.position.x<1-h && p.position.y<=h) {bottomAccel2+=acceleration*acceleration;++bottom;}
        if(p.position.x<=-1+h && p.position.y<=h) {cornerAccel2+=acceleration*acceleration;++corner;}
        if(p.position.x>-1+h && p.position.x<1-h && p.position.y>h && p.position.y<1.5-h) {bulkAccel2+=acceleration*acceleration;++bulk;}
    }
    if(maxReconstruction/std::max(scaleForce,1.0)>2e-5)
        throw std::runtime_error("Independent wall pressure reconstruction disagrees with prepare().");
    out << "{\"geometry\":\"" << geometry << "\",\"initialization\":\"" << (initialized?"exact Tait hydrostatic EOS / continuity":"uninitialized nominal summation")
        << "\",\"family\":\"" << (family==SphKernelFamily::CubicSpline?"cubic":"legacy") << "\",\"refinementPath\":\"" << path
        << "\",\"spacing\":" << dx << ",\"h\":" << h << ",\"hOverDx\":" << h/dx << ",\"particles\":" << particles.size()
        << ",\"wallSamples\":" << walls.size() << ",\"pressurePairs\":" << op.pairs.size() << ",\"activeWallPairs\":" << op.walls.size()
        << ",\"pressureNormalizedRmse\":" << std::sqrt(pressureError/particles.size()) << ",\"accelerationRms\":" << std::sqrt(accel2/particles.size())
        << ",\"flatWallInteriorAccelerationRms\":" << (bottom?std::sqrt(bottomAccel2/bottom):0)
        << ",\"bottomLeftRegionAccelerationRms\":" << (corner?std::sqrt(cornerAccel2/corner):0)
        << ",\"bulkAccelerationRms\":" << (bulk?std::sqrt(bulkAccel2/bulk):0) << ",\"bulkAccelerationSampleCount\":" << bulk
        << ",\"normalizedVerticalForceResidual\":" << std::abs(actual.y)/std::abs(gravity.y)
        << ",\"maximumForceReconstructionError\":" << maxReconstruction << ",\"relativeForceReconstructionError\":" << maxReconstruction/std::max(scaleForce,1.0)
        << ",\"fluidPairForce\":";WriteVector(out,pair);out << ",\"wallForce\":";WriteVector(out,wall);
    out << ",\"gravityForce\":";WriteVector(out,gravity);out << ",\"inferredWallReaction\":";WriteVector(out,wall*-1);
    out << ",\"signedExtrapolationCounterfactualWallForce\":";WriteVector(out,signedWall);
    out << ",\"signedExtrapolationCounterfactualVerticalResidual\":" << std::abs((gravity+signedWall).y)/std::abs(gravity.y)
        << ",\"signedExtrapolationCounterfactualAccelerationRms\":" << std::sqrt(signedAccel2/particles.size());
    out << ",\"localAdjointCounterfactualWallForce\":";WriteVector(out,localAdjointWall);
    out << ",\"localAdjointCounterfactualVerticalResidual\":" << std::abs((gravity+localAdjointWall).y)/std::abs(gravity.y);
    out << ",\"preparedFluidForce\":";WriteVector(out,actual);out << ",\"responses\":{";
    bool first=true;
    for(const char* mode:{"rest","tangent_fixed_wall","translation_with_wall","normal_fixed_wall","isotropic_compression","rigid_rotation_with_wall"}) {
        if(!first) out << ',';first=false;
        for(auto& p:particles) {
            p.velocity={};
            if(std::string(mode)=="tangent_fixed_wall") p.velocity={0.2f,0};
            if(std::string(mode)=="translation_with_wall") p.velocity={0.2f,0.1f};
            if(std::string(mode)=="normal_fixed_wall") p.velocity={0,-0.1f};
            if(std::string(mode)=="isotropic_compression") p.velocity=p.position*-0.1f;
            if(std::string(mode)=="rigid_rotation_with_wall") p.velocity={-0.1f*p.position.y,0.1f*p.position.x};
        }
        for(auto& b:walls) {
            b.velocity=std::string(mode)=="translation_with_wall"?Vector2{0.2f,0.1f}:Vector2{};
            if(std::string(mode)=="rigid_rotation_with_wall") b.velocity={-0.1f*b.position.y,0.1f*b.position.x};
        }
        out << '\"' << mode << "\":";WriteWork(out,MeasureWork(op,particles,walls),particles);
    }
    out << "}}";
}
}
int main(int argc,char** argv) {
    try {
        bool quick=false;std::string path;
        for(int i=1;i<argc;++i) {
            const std::string argument=argv[i];
            if(argument=="--quick") quick=true;
            else if(argument=="--output" && i+1<argc) path=argv[++i];
            else throw std::invalid_argument("Usage: fluid_wall_diagnostic [--quick] [--output report.json]");
        }
        std::ofstream file;
        if(!path.empty()) {file.open(path);if(!file) throw std::runtime_error("Cannot open wall diagnostic output.");}
        auto& out=file.is_open()?static_cast<std::ostream&>(file):std::cout;
        out << std::setprecision(12) << "{\"schemaVersion\":1,\"quick\":" << (quick?"true":"false")
            << ",\"model\":\"prepared WCSPH radial sampled-wall pressure / continuity audit\","
            << "\"settings\":{\"rho0\":1000,\"speedOfSound\":40,\"gamma\":7,\"gravity\":[0,-9.81],\"surfaceHeight\":1.5,"
            << "\"massPolicy\":\"nominal rho0*dx^2, no calibration\",\"viscosity\":0,\"densityDiffusion\":0,\"pressureScale\":1},"
            << "\"constantPressurePhaseSettings\":{\"rho0\":1000,\"rho\":1010,\"speedOfSound\":20,\"gamma\":7,"
            << "\"gravity\":[0,0],\"massPolicy\":\"explicit rho*dx^2 for equal quadrature volumes\"},\"cases\":[";
        bool first=true;
        for(const char* geometry:{"flat","corner","column"})
            for(auto family:{SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline}) for(bool initialized:{false,true}) {
                const auto write=[&](float dx,float h,const char* refinement) {if(!first) out << ',';first=false;WriteScene(out,geometry,dx,h,family,initialized,refinement);};
                write(0.1f,0.25f,"fixed-ratio coarse");
                if(!quick) {
                    write(0.05f,0.125f,"fixed-ratio");write(0.025f,0.0625f,"fixed-ratio");
                    write(0.05f,0.25f,"fixed-h neighbor-refinement");write(0.025f,0.25f,"fixed-h neighbor-refinement");
                    write(0.1f,0.2f,"fixed-dx ratio");write(0.1f,0.4f,"fixed-dx ratio");
                }
            }
        out << "],\"constantPressureWallPhaseControls\":[";
        first=true;
        for(auto family:{SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline})
            for(float dx:{0.1f,0.05f,0.025f}) for(float ratio:{2.0f,2.5f,4.0f}) for(bool staggered:{false,true}) {
                if(!first) out << ',';first=false;
                const auto m=FlatPhaseProbe(family,dx,ratio,staggered);
                out << "{\"family\":\"" << (family==SphKernelFamily::CubicSpline?"cubic":"legacy")
                    << "\",\"spacing\":" << dx << ",\"hOverDx\":" << ratio << ",\"tangentialPhaseOverDx\":" << (staggered?0.5:0)
                    << ",\"normalizedNormalForceOverPressureDx\":" << m.normalizedNormalForce
                    << ",\"normalAcceleration\":" << m.normalAcceleration << ",\"uniformPressure\":" << m.pressure << '}';
            }
        out << "]}\n";out.flush();if(!out) throw std::runtime_error("Cannot write wall diagnostic output.");
    } catch(const std::exception& e) {std::cerr << e.what() << '\n';return 1;}
}
