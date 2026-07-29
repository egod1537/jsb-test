#include "application/sim/control/ManualControlInputStrategy.hpp"

namespace control {
const char *ManualControlInputStrategy::GetName() const { return "Manual"; }

void ManualControlInputStrategy::Reset() { commandedInput_ = {}; }

const ControlInput &ManualControlInputStrategy::GetCommandedInput() const {
  return commandedInput_;
}

bool ManualControlInputStrategy::SetCommandedInput(ControlAxis axis,
    double value) {
  return SetControlAxisValue(commandedInput_, axis, value);
}

bool ManualControlInputStrategy::Update(const sim::Aircraft &, double,
    ControlInput &output) {
  const ControlInput previousOutput = output;
  output = commandedInput_;
  ClampControlInput(output);
  return output != previousOutput;
}
} // namespace control
