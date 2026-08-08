#pragma once

#include "application/sim/gnc/hold/PitchDynamics.hpp"
#include "application/sim/gnc/hold/RollDynamics.hpp"

#include <optional>

namespace gnc {
struct ControlContext {
  std::optional<RollDynamics> rollDynamics;
  std::optional<PitchDynamics> pitchDynamics;
};
} // namespace gnc
