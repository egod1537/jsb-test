#pragma once

#include "application/sim/control/FlightControlController.hpp"

namespace control {
class ManualFlightControlController final : public FlightControlController {
public:
  const char *GetName() const override;
  void Reset() override;
  ControlInput Update(const sim::Aircraft &aircraft, double dt) override;

  const ControlInput &GetCommandedInput() const;
  bool SetCommandedInput(const ControlInput &input);
  bool SetCommandedInput(ControlAxis axis, double value);
  bool AdjustCommandedInput(ControlAxis axis, double delta);

private:
  ControlInput commandedInput_;
};
} // namespace control
