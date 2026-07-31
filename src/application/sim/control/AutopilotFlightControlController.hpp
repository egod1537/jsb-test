#pragma once

#include "application/sim/control/FlightControlController.hpp"

namespace gnc {
class Autopilot;
} // namespace gnc

namespace control {
class ManualFlightControlController;

class AutopilotFlightControlController final : public FlightControlController {
public:
  AutopilotFlightControlController(gnc::Autopilot &autopilot,
      const ManualFlightControlController &manualController);

  const char *GetName() const override;
  void Reset() override;
  ControlInput Update(const sim::Aircraft &aircraft, double dt) override;

private:
  gnc::Autopilot &autopilot_;
  const ManualFlightControlController &manualController_;
};
} // namespace control
