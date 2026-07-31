#include "application/sim/control/ManualFlightControlController.hpp"

namespace control {
const char *ManualFlightControlController::GetName() const {
  return "Manual";
}

void ManualFlightControlController::Reset() { commandedInput_ = {}; }

ControlInput ManualFlightControlController::Update(
    const sim::Aircraft &, double) {
  return commandedInput_;
}

const ControlInput &ManualFlightControlController::GetCommandedInput()
    const {
  return commandedInput_;
}

bool ManualFlightControlController::SetCommandedInput(
    const ControlInput &input) {
  bool changed = false;
  changed = SetCommandedInput(ControlAxis::Elevator, input.elevator)
            || changed;
  changed = SetCommandedInput(ControlAxis::Aileron, input.aileron)
            || changed;
  changed = SetCommandedInput(ControlAxis::Rudder, input.rudder) || changed;
  changed = SetCommandedInput(ControlAxis::Throttle, input.throttle)
            || changed;
  return changed;
}

bool ManualFlightControlController::SetCommandedInput(ControlAxis axis,
    double value) {
  return SetControlAxisValue(commandedInput_, axis, value);
}

bool ManualFlightControlController::AdjustCommandedInput(ControlAxis axis,
    double delta) {
  return AdjustControlAxisValue(commandedInput_, axis, delta);
}
} // namespace control
