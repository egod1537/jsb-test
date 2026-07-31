#include "application/sim/control/AutopilotFlightControlController.hpp"

#include "application/sim/control/ManualFlightControlController.hpp"
#include "application/sim/gnc/Autopilot.hpp"

namespace control {
AutopilotFlightControlController::AutopilotFlightControlController(
    gnc::Autopilot &autopilot,
    const ManualFlightControlController &manualController)
    : autopilot_(autopilot), manualController_(manualController) {}

const char *AutopilotFlightControlController::GetName() const {
  return "Autopilot";
}

void AutopilotFlightControlController::Reset() { autopilot_.Reset(); }

ControlInput AutopilotFlightControlController::Update(
    const sim::Aircraft &aircraft, double dt) {
  ControlInput input = manualController_.GetCommandedInput();

  if (auto aileron =
          autopilot_.GetRollHoldController().Update(aircraft, dt)) {
    input.aileron = *aileron;
  }

  if (auto elevator =
          autopilot_.GetPitchHoldController().Update(aircraft, dt)) {
    input.elevator = *elevator;
  }

  ClampControlInput(input);
  return input;
}
} // namespace control
