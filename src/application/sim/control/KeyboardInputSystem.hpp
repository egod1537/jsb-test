#pragma once

#include "application/sim/control/KeyboardInput.hpp"
#include "application/sim/System.hpp"

namespace control {
class KeyboardInputSystem final : public sim::System {
public:
  bool Initialize(sim::Context &context) override;
  bool PreStep(sim::Context &context, const sim::Tick &tick) override;
  void Shutdown(sim::Context &context) override;

private:
  KeyboardInput keyboardInput_;
};
} // namespace control
