#include "control/KeyboardInputSystem.hpp"

#include "control/ControlInputStrategy.hpp"
#include "simulation/Aircraft.hpp"
#include "simulation/Context.hpp"
#include "simulation/Tick.hpp"

#include <iostream>

namespace control {
bool KeyboardInputSystem::Initialize(sim::Context &context) {
  if (!keyboardInput_.Initialize()) {
    context.SetError("Failed to initialize keyboard input.");
    std::cerr << "Failed to initialize keyboard input.\n";
    return false;
  }

  return true;
}

bool KeyboardInputSystem::PreStep(sim::Context &context, const sim::Tick &) {
  ControlInputStrategy &strategy =
      context.GetAircraft().GetControlInputStrategy();
  if (keyboardInput_.Update(strategy)) {
    const ControlInput &controlInput = strategy.GetCommandedInput();
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
