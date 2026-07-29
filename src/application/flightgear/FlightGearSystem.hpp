#pragma once

#include "application/flightgear/FlightGearSender.hpp"
#include "application/sim/System.hpp"

namespace flightgear {
class FlightGearSystem final : public sim::System {
public:
  bool Initialize(sim::Context &context) override;
  bool Reset(sim::Context &context) override;
  bool PostStep(sim::Context &context, const sim::Tick &tick) override;

private:
  bool Send(sim::Context &context);

  FlightGearSender sender_;
};
} // namespace flightgear
