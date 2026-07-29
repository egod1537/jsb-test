#include "flightgear/FlightGearSystem.hpp"

#include "simulation/Aircraft.hpp"
#include "simulation/Context.hpp"
#include "simulation/Tick.hpp"

#include <iostream>

namespace flightgear {
bool FlightGearSystem::Initialize(sim::Context &context) {
  if (!sender_.IsOpen()) {
    context.SetError("Failed to initialize FlightGear sender.");
    std::cerr << "Failed to initialize FlightGear sender.\n";
    return false;
  }

  return true;
}

bool FlightGearSystem::Reset(sim::Context &context) { return Send(context); }

bool FlightGearSystem::PostStep(sim::Context &context, const sim::Tick &) {
  return Send(context);
}

bool FlightGearSystem::Send(sim::Context &context) {
  if (!sender_.Send(context.GetAircraft().GetFDMExec())) {
    std::cerr << "Failed to send FlightGear packet\n";
  }

  return true;
}
} // namespace flightgear
