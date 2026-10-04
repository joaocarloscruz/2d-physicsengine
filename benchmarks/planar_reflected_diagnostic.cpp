#include "planar_reflected_operator.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
using namespace PlanarReflection;
using PhysicsEngine::SphKernelFamily;
namespace {
struct Sample { std::vector<Particle> p; std::size_t bottom=0,bulk=0; };
Sample Lattice(double dx,double ratio,const std::string& control) {
    const int extent=static_cast<int>(std::ceil(ratio))+2,rows=2*extent+1;
    Sample s;
    for(int y=0;y<rows;++y) for(int x=-extent;x<=extent;++x) {
        Vec position{x*dx,(y+.5)*dx};
        if(control=="common_phase") position.x+=.37*dx;
        if(control=="row_phase") position.x+=(y%2)*.5*dx;
        if(control=="disorder") {
            position.x+=.12*dx*std::sin(1.7*x+2.3*y+.4);
            position.y+=.12*dx*std::cos(2.1*x-1.3*y+.7);
        }
        if(x==0&&y==0) s.bottom=s.p.size();
        if(x==0&&y==extent) s.bulk=s.p.size();
        s.p.push_back({position,{},1000*dx*dx,1000,1000});
    }
    return s;
}
void Write(std::ostream& stream,SphKernelFamily family,double dx,double ratio,const std::string& control,const std::string& refinement) {
    const auto sample=Lattice(dx,ratio,control); Plane plane;
    const auto out=Evaluate(sample.p,plane,dx*ratio,family);
    auto compress=sample.p;
    for(auto& p:compress) p.velocity=p.position*(-.1);
    const auto compression=Evaluate(compress,plane,dx*ratio,family);
    auto tangent=sample.p; for(auto& p:tangent) p.velocity={.4,0};
    const auto tangentOut=Evaluate(tangent,plane,dx*ratio,family);
    auto rotation=sample.p; plane.center={.12,-.17}; plane.velocity={.2,-.3}; plane.angularVelocity=.7;
    for(auto& p:rotation) p.velocity=plane.VelocityAt(p.position);
    const auto rigid=Evaluate(rotation,plane,dx*ratio,family);
    double tangentRate=0,rigidRate=0;
    for(std::size_t i=0;i<sample.p.size();++i) {
        tangentRate=std::max(tangentRate,std::abs(tangentOut.densityRates[i]/sample.p[i].density));
        rigidRate=std::max(rigidRate,std::abs(rigid.densityRates[i]/sample.p[i].density));
    }
    const double scale=1000*dx;
    const auto normalized=out.forces[sample.bottom]*(1/scale);
    const double residual=std::abs(compression.workResidual)/(1+std::abs(compression.fluidWork)+std::abs(compression.wallWork)+std::abs(compression.internalEnergyRate));
    // Algebraic acceptance only: quadrature deficiencies are measured, not hidden.
    if(residual>1e-12||rigidRate>1e-12||tangentRate>1e-12||out.totalForce.norm()>1e-7||std::abs(out.totalTorque)>1e-7)
        throw std::runtime_error("Reflected operator identity failed.");
    stream<<"{\"family\":\""<<(family==SphKernelFamily::CubicSpline?"cubic":"legacy")
        <<"\",\"control\":\""<<control<<"\",\"refinement\":\""<<refinement
        <<"\",\"dx\":"<<dx<<",\"h_over_dx\":"<<ratio<<",\"particles\":"<<sample.p.size()
        <<",\"bottom_density_ratio\":"<<out.summedDensities[sample.bottom]/1000
        <<",\"bottom_force_over_p_dx_x\":"<<normalized.x<<",\"bottom_force_over_p_dx_y\":"<<normalized.y
        <<",\"bottom_acceleration\":"<<out.forces[sample.bottom].norm()/sample.p[sample.bottom].mass
        <<",\"bulk_acceleration\":"<<out.forces[sample.bulk].norm()/sample.p[sample.bulk].mass
        <<",\"compression_bottom_rate_over_rho\":"<<compression.densityRates[sample.bottom]/1000
        <<",\"compression_bulk_rate_over_rho\":"<<compression.densityRates[sample.bulk]/1000
        <<",\"fluid_pair_force_x\":"<<out.pairForces[sample.bottom].x<<",\"fluid_pair_force_y\":"<<out.pairForces[sample.bottom].y
        <<",\"reflected_source_force_x\":"<<out.ghostForces[sample.bottom].x<<",\"reflected_source_force_y\":"<<out.ghostForces[sample.bottom].y
        <<",\"wall_force_x\":"<<out.wallForce.x<<",\"wall_force_y\":"<<out.wallForce.y
        <<",\"wall_torque_about_origin\":"<<out.wallTorque<<",\"wall_projection_torque\":"<<out.wallProjectionTorque
        <<",\"wall_couple\":"<<out.wallCouple<<",\"total_force_norm\":"<<out.totalForce.norm()<<",\"total_torque\":"<<out.totalTorque
        <<",\"compression_fluid_work\":"<<compression.fluidWork<<",\"compression_wall_work\":"<<compression.wallWork
        <<",\"compression_internal_energy_rate\":"<<compression.internalEnergyRate<<",\"relative_work_residual\":"<<residual
        <<",\"max_tangent_rate_over_rho\":"<<tangentRate<<",\"max_joint_rigid_rate_over_rho\":"<<rigidRate<<"}";
}
}
int main(int argc,char** argv) {
    try {
        std::string path; bool quick=false;
        for(int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            if(arg=="--quick") quick=true;
            else if(arg=="--output"&&i+1<argc) path=argv[++i];
            else throw std::invalid_argument("Usage: planar_reflected_diagnostic [--quick] [--output file.json]");
        }
        std::ofstream file; std::ostream* output=&std::cout;
        if(!path.empty()) { file.open(path); if(!file) throw std::runtime_error("Cannot open diagnostic output."); output=&file; }
        auto& stream=*output; stream<<std::setprecision(17)
            <<"{\"schema\":1,\"scope\":\"static single plane; no integration or calibration\",\"rho\":1000,\"pressure\":1000,\"compression_rate\":0.1,\"cases\":[\n";
        bool first=true;
        auto emit=[&](SphKernelFamily f,double dx,double ratio,const std::string& control,const std::string& refinement) {
            if(!first) stream<<",\n"; first=false; Write(stream,f,dx,ratio,control,refinement);
        };
        for(auto family:{SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline})
            for(const std::string control:{"aligned","common_phase","row_phase","disorder"}) {
                for(double dx:quick?std::vector<double>{.1}:std::vector<double>{.1,.05,.025}) emit(family,dx,2.5,control,"fixed_ratio");
                if(!quick) {
                    for(double ratio:{2.,4.,8.}) emit(family,.1,ratio,control,"fixed_dx");
                    for(double dx:{.1,.05,.025}) emit(family,dx,.2/dx,control,"fixed_h");
                }
            }
        stream<<"\n]}\n"; stream.flush(); if(!stream) throw std::runtime_error("Cannot write diagnostic output.");
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
