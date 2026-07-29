#include "gnc/RollHoldSystem.hpp"

#include "simulation/Context.hpp"
#include "simulation/Tick.hpp"

namespace gnc {
bool RollHoldSystem::PreStep(sim::Context &, const sim::Tick &) {
  return true;
}
} // namespace gnc
