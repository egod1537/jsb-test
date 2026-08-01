#pragma once

#include "application/sim/control/IFlightControlSource.hpp"

namespace control {
class ManualFlightControlController final : public IFlightControlSource {
public:
  void OnReset();
  ControlInput OnTick(const sim::Aircraft &aircraft,
      const sim::Tick &tick) override;

  const ControlInput &GetCommandedInput() const;
  bool SetCommandedInput(const ControlInput &input);
  bool SetCommandedInput(ControlAxis axis, double value);
  bool AdjustCommandedInput(ControlAxis axis, double delta);

private:
  ControlInput commandedInput_;
};
} // namespace control
