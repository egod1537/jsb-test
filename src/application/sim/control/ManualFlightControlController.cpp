#include "application/sim/control/ManualFlightControlController.hpp"

#include "application/input/Input.hpp"

#include <iostream>

namespace control {
namespace {
constexpr double InputStep = 0.05;
}

void ManualFlightControlController::OnReset() { commandedInput_ = {}; }

ControlInput ManualFlightControlController::OnTick(sim::Aircraft &,
    const sim::Tick &) {
  bool changed = false;

  if (application::Input::IsKeyPressed(application::Key::W)) {
    changed =
        AdjustCommandedInput(ControlAxis::Elevator, -InputStep) || changed;
  }
  if (application::Input::IsKeyPressed(application::Key::S)) {
    changed = AdjustCommandedInput(ControlAxis::Elevator, InputStep) || changed;
  }
  if (application::Input::IsKeyPressed(application::Key::A)) {
    changed = AdjustCommandedInput(ControlAxis::Aileron, -InputStep) || changed;
  }
  if (application::Input::IsKeyPressed(application::Key::D)) {
    changed = AdjustCommandedInput(ControlAxis::Aileron, InputStep) || changed;
  }
  if (application::Input::IsKeyPressed(application::Key::Q)) {
    changed = AdjustCommandedInput(ControlAxis::Rudder, -InputStep) || changed;
  }
  if (application::Input::IsKeyPressed(application::Key::E)) {
    changed = AdjustCommandedInput(ControlAxis::Rudder, InputStep) || changed;
  }
  if (application::Input::IsKeyPressed(application::Key::R)) {
    changed = AdjustCommandedInput(ControlAxis::Throttle, InputStep) || changed;
  }
  if (application::Input::IsKeyPressed(application::Key::F)) {
    changed =
        AdjustCommandedInput(ControlAxis::Throttle, -InputStep) || changed;
  }

  if (changed) {
    std::cout << "control"
              << " elevator=" << commandedInput_.elevator
              << " aileron=" << commandedInput_.aileron
              << " rudder=" << commandedInput_.rudder
              << " throttle=" << commandedInput_.throttle << '\n';
  }

  return commandedInput_;
}

const ControlInput &ManualFlightControlController::GetCommandedInput() const {
  return commandedInput_;
}

bool ManualFlightControlController::SetCommandedInput(
    const ControlInput &input) {
  bool changed = false;
  changed = SetCommandedInput(ControlAxis::Elevator, input.elevator) || changed;
  changed = SetCommandedInput(ControlAxis::Aileron, input.aileron) || changed;
  changed = SetCommandedInput(ControlAxis::Rudder, input.rudder) || changed;
  changed = SetCommandedInput(ControlAxis::Throttle, input.throttle) || changed;
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
