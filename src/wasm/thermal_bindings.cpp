#include <emscripten/bind.h>
#include "physics/core/thermal_network.h"
#include "checked_indices.h"

namespace PhysicsEngine::Wasm {
namespace {
struct ThermalConfig {
    double maxSubstep, safetyFactor, maxSubsteps, maxNodes, maxLinks;
};
ThermalNetworkConfig Native(ThermalConfig c) {
    return {c.maxSubstep,c.safetyFactor,Count(c.maxSubsteps),Count(c.maxNodes),Count(c.maxLinks)};
}
ThermalNetwork* Create(ThermalConfig c) { return new ThermalNetwork(Native(c)); }
ThermalConfig Config(const ThermalNetwork& s) {
    const auto& c=s.getConfig();
    return {c.maxSubstep,c.safetyFactor,double(c.maxSubsteps),double(c.maxNodes),double(c.maxLinks)};
}
std::size_t NodeIndex(const ThermalNetwork& s,double index) { return Index(index,s.getNodes().size()); }
}
}

EMSCRIPTEN_BINDINGS(thermal_network) {
    using namespace emscripten;
    using namespace PhysicsEngine;
    using namespace PhysicsEngine::Wasm;
    value_object<ThermalConfig>("ThermalNetworkConfig")
        .field("maxSubstep",&ThermalConfig::maxSubstep).field("safetyFactor",&ThermalConfig::safetyFactor)
        .field("maxSubsteps",&ThermalConfig::maxSubsteps).field("maxNodes",&ThermalConfig::maxNodes)
        .field("maxLinks",&ThermalConfig::maxLinks);
    value_object<ThermalNode>("ThermalNode")
        .field("temperature",&ThermalNode::temperature).field("heatCapacity",&ThermalNode::heatCapacity)
        .field("fixed",&ThermalNode::fixed).field("externalPower",&ThermalNode::externalPower)
        .field("reservoirHeat",&ThermalNode::reservoirHeat);
    value_object<ThermalLink>("ThermalLink")
        .field("first",&ThermalLink::first).field("second",&ThermalLink::second)
        .field("conductance",&ThermalLink::conductance);
    value_object<ThermalDiagnostics>("ThermalDiagnostics")
        .field("totalEnergy",&ThermalDiagnostics::totalEnergy)
        .field("minimumTemperature",&ThermalDiagnostics::minimumTemperature)
        .field("maximumTemperature",&ThermalDiagnostics::maximumTemperature)
        .field("totalExternalEnergy",&ThermalDiagnostics::totalExternalEnergy)
        .field("totalReservoirHeat",&ThermalDiagnostics::totalReservoirHeat)
        .field("lastExternalEnergy",&ThermalDiagnostics::lastExternalEnergy)
        .field("lastReservoirHeat",&ThermalDiagnostics::lastReservoirHeat)
        .field("lastSubsteps",&ThermalDiagnostics::lastSubsteps);
    class_<ThermalNetwork>("ThermalNetwork")
        .constructor<>().constructor(&Create,allow_raw_pointers())
        .function("getConfig",&Config)
        .function("setConfig",optional_override([](ThermalNetwork& s,ThermalConfig c) { s.setConfig(Native(c)); }))
        .function("getNodeCount",optional_override([](const ThermalNetwork& s) { return s.getNodes().size(); }))
        .function("getLinkCount",optional_override([](const ThermalNetwork& s) { return s.getLinks().size(); }))
        .function("getNode",optional_override([](const ThermalNetwork& s,double index) { return ThermalNode(s.getNodes().at(NodeIndex(s,index))); }))
        .function("getLink",optional_override([](const ThermalNetwork& s,double index) { return ThermalLink(s.getLinks().at(Index(index,s.getLinks().size()))); }))
        .function("addNode",optional_override([](ThermalNetwork& s,double temperature,double capacity) { return s.addNode(temperature,capacity); }))
        .function("addNode",&ThermalNetwork::addNode)
        .function("addLink",optional_override([](ThermalNetwork& s,double first,double second,double conductance) { return s.addLink(NodeIndex(s,first),NodeIndex(s,second),conductance); }))
        .function("setTemperature",optional_override([](ThermalNetwork& s,double index,double temperature) { s.setTemperature(NodeIndex(s,index),temperature); }))
        .function("setFixed",optional_override([](ThermalNetwork& s,double index,bool fixed) { s.setFixed(NodeIndex(s,index),fixed); }))
        .function("applyPower",optional_override([](ThermalNetwork& s,double index,double power) { s.applyPower(NodeIndex(s,index),power); }))
        .function("clearPowers",select_overload<void()>(&ThermalNetwork::clearPowers))
        .function("clearPowers",optional_override([](ThermalNetwork& s,double index) { s.clearPowers(NodeIndex(s,index)); }))
        .function("getDiagnostics",&ThermalNetwork::getDiagnostics)
        .function("step",&ThermalNetwork::step);
}
