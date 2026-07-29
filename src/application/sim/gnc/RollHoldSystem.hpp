#pragma once

#include "application/sim/System.hpp"

namespace gnc {
class RollHoldSystem final : public sim::System {
public:
  bool PreStep(sim::Context &context, const sim::Tick &tick) override;
};
} // namespace gnc
