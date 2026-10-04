#include <emscripten/bind.h>
#include "physics/core/wave_membrane.h"
#include "checked_indices.h"
#include <stdexcept>
#include <string>

namespace PhysicsEngine::Wasm {
namespace {
struct WaveConfig {
    double tension, surfaceDensity, damping;
    WaveBoundary boundary;
    double cflSafety, maxSubstep, maxCells, maxSubsteps, maxCellWork;
};
struct WaveCell { double displacement, velocity, queuedAcceleration; };
WaveMembraneConfig Native(WaveConfig c) {
    return {c.tension,c.surfaceDensity,c.damping,c.boundary,c.cflSafety,c.maxSubstep,
        Count(c.maxCells),Count(c.maxSubsteps),Count(c.maxCellWork)};
}
WaveConfig Config(const WaveMembrane& s) {
    const auto& c=s.getConfig();
    return {c.tension,c.surfaceDensity,c.damping,c.boundary,c.cflSafety,c.maxSubstep,
        double(c.maxCells),double(c.maxSubsteps),double(c.maxCellWork)};
}
WaveMembrane* CreateDefault(double width,double height,double dx,double dy) {
    return new WaveMembrane(Count(width),Count(height),dx,dy);
}
WaveMembrane* CreateConfigured(double width,double height,double dx,double dy,WaveConfig c) {
    return new WaveMembrane(Count(width),Count(height),dx,dy,Native(c));
}
std::size_t CellIndex(const WaveMembrane& s,double x,double y) {
    return Index(y,s.getHeight())*s.getWidth()+Index(x,s.getWidth());
}
WaveCell Cell(const WaveMembrane& s,double x,double y) {
    const auto i=CellIndex(s,x,y);
    return {s.getDisplacements()[i],s.getVelocities()[i],s.getQueuedAccelerations()[i]};
}
emscripten::val Copy(const std::vector<double>& values) {
    auto result=emscripten::val::array();
    for(std::size_t i=0;i<values.size();++i) result.set(i,values[i]);
    return result;
}
void SetState(WaveMembrane& s,const emscripten::val& u,const emscripten::val& v) {
    using emscripten::val;
    const auto array=val::global("Array");
    if (!array.call<bool>("isArray",u) || !array.call<bool>("isArray",v))
        throw std::invalid_argument("Wave state requires plain JavaScript arrays");
    const auto count=s.getDisplacements().size();
    // Check both shapes before allocating either bounded native copy.
    if (Count(u["length"].as<double>())!=count || Count(v["length"].as<double>())!=count)
        throw std::invalid_argument("Wave state array length does not match the grid");
    std::vector<double> newU(count),newV(count);
    for(std::size_t i=0;i<count;++i) {
        const auto ui=u[i],vi=v[i];
        if (ui.typeOf().as<std::string>()!="number" || vi.typeOf().as<std::string>()!="number")
            throw std::invalid_argument("Wave state entries must be numbers");
        newU[i]=ui.as<double>(); newV[i]=vi.as<double>();
    }
    s.setState(newU,newV);
}
}
}

EMSCRIPTEN_BINDINGS(wave_membrane) {
    using namespace emscripten;
    using namespace PhysicsEngine;
    using namespace PhysicsEngine::Wasm;
    enum_<WaveBoundary>("WaveBoundary")
        .value("FixedZero",WaveBoundary::FixedZero).value("Periodic",WaveBoundary::Periodic);
    value_object<WaveConfig>("WaveMembraneConfig")
        .field("tension",&WaveConfig::tension).field("surfaceDensity",&WaveConfig::surfaceDensity)
        .field("damping",&WaveConfig::damping).field("boundary",&WaveConfig::boundary)
        .field("cflSafety",&WaveConfig::cflSafety).field("maxSubstep",&WaveConfig::maxSubstep)
        .field("maxCells",&WaveConfig::maxCells).field("maxSubsteps",&WaveConfig::maxSubsteps)
        .field("maxCellWork",&WaveConfig::maxCellWork);
    value_object<WaveCell>("WaveCell")
        .field("displacement",&WaveCell::displacement).field("velocity",&WaveCell::velocity)
        .field("queuedAcceleration",&WaveCell::queuedAcceleration);
    value_object<WaveMembraneDiagnostics>("WaveMembraneDiagnostics")
        .field("kineticEnergy",&WaveMembraneDiagnostics::kineticEnergy)
        .field("strainEnergy",&WaveMembraneDiagnostics::strainEnergy).field("totalEnergy",&WaveMembraneDiagnostics::totalEnergy)
        .field("maxAbsDisplacement",&WaveMembraneDiagnostics::maxAbsDisplacement)
        .field("maxAbsVelocity",&WaveMembraneDiagnostics::maxAbsVelocity).field("time",&WaveMembraneDiagnostics::time)
        .field("stableTimeStep",&WaveMembraneDiagnostics::stableTimeStep).field("lastSubstep",&WaveMembraneDiagnostics::lastSubstep)
        .field("lastSubsteps",&WaveMembraneDiagnostics::lastSubsteps).field("lastCellWork",&WaveMembraneDiagnostics::lastCellWork);
    class_<WaveMembrane>("WaveMembrane")
        .constructor(&CreateDefault,allow_raw_pointers()).constructor(&CreateConfigured,allow_raw_pointers())
        .function("getWidth",&WaveMembrane::getWidth).function("getHeight",&WaveMembrane::getHeight)
        .function("getSpacingX",&WaveMembrane::getSpacingX).function("getSpacingY",&WaveMembrane::getSpacingY)
        .function("getCellCount",optional_override([](const WaveMembrane& s) { return s.getDisplacements().size(); }))
        .function("getConfig",&Config)
        .function("setConfig",optional_override([](WaveMembrane& s,WaveConfig c) { s.setConfig(Native(c)); }))
        .function("getCell",&Cell)
        .function("getDisplacements",optional_override([](const WaveMembrane& s) { return Copy(s.getDisplacements()); }))
        .function("getVelocities",optional_override([](const WaveMembrane& s) { return Copy(s.getVelocities()); }))
        .function("getQueuedAccelerations",optional_override([](const WaveMembrane& s) { return Copy(s.getQueuedAccelerations()); }))
        .function("setState",&SetState)
        .function("setCellState",optional_override([](WaveMembrane& s,double x,double y,double u) {
            CellIndex(s,x,y); s.setCellState(Count(x),Count(y),u);
        }))
        .function("setCellState",optional_override([](WaveMembrane& s,double x,double y,double u,double v) {
            CellIndex(s,x,y); s.setCellState(Count(x),Count(y),u,v);
        }))
        .function("queueAcceleration",optional_override([](WaveMembrane& s,double x,double y,double a) {
            CellIndex(s,x,y); s.queueAcceleration(Count(x),Count(y),a);
        }))
        .function("clearAcceleration",optional_override([](WaveMembrane& s,double x,double y) {
            CellIndex(s,x,y); s.clearAcceleration(Count(x),Count(y));
        }))
        .function("clearAccelerations",&WaveMembrane::clearAccelerations)
        .function("getStableTimeStep",&WaveMembrane::getStableTimeStep)
        .function("getDiagnostics",&WaveMembrane::getDiagnostics).function("step",&WaveMembrane::step);
}
