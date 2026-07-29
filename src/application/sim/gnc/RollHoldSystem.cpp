#include "application/sim/gnc/RollHoldSystem.hpp"

#include "application/sim/Context.hpp"
#include "application/sim/Tick.hpp"

namespace gnc {
bool RollHoldSystem::PreStep(sim::Context &, const sim::Tick &) {
  return true;
}
} // namespace gnc
