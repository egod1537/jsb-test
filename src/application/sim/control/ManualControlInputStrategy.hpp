#pragma once

#include "application/sim/control/ControlInputStrategy.hpp"

namespace control {
class ManualControlInputStrategy final : public ControlInputStrategy {
public:
  const char *GetName() const override;
  void Reset() override;
  const ControlInput &GetCommandedInput() const override;
  bool SetCommandedInput(ControlAxis axis, double value) override;
  bool Update(const sim::Aircraft &aircraft, double dt,
      ControlInput &output) override;

private:
  ControlInput commandedInput_;
};
} // namespace control
