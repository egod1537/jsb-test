#include "application/sim/control/KeyboardInputSystem.hpp"

#include "application/sim/control/ManualFlightControlController.hpp"
#include "application/sim/Context.hpp"
#include "application/sim/Tick.hpp"

#include <iostream>

namespace control {
void KeyboardInputSystem::SetManualFlightControlController(
    ManualFlightControlController &manualController) {
  manualController_ = &manualController;
}

bool KeyboardInputSystem::Initialize(sim::Context &context) {
  if (!keyboardInput_.Initialize()) {
    context.SetError("Failed to initialize keyboard input.");
    std::cerr << "Failed to initialize keyboard input.\n";
    return false;
  }

  return true;
}

bool KeyboardInputSystem::PreStep(sim::Context &context, const sim::Tick &) {
  if (manualController_ == nullptr) {
    context.SetError("Keyboard input has no manual flight control controller.");
    return false;
  }

  if (keyboardInput_.Update(*manualController_)) {
    const ControlInput &controlInput = manualController_->GetCommandedInput();
    std::cout << "control"
              << " elevator=" << controlInput.elevator
              << " aileron=" << controlInput.aileron
              << " rudder=" << controlInput.rudder
              << " throttle=" << controlInput.throttle << '\n';
  }

  return true;
}

void KeyboardInputSystem::Shutdown(sim::Context &) { keyboardInput_.Shutdown(); }
} // namespace control
