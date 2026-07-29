#pragma once

#include "application/sim/control/ControlInput.hpp"

namespace sim {
class Aircraft;
} // namespace sim

namespace control {
class ControlInputStrategy {
public:
  virtual ~ControlInputStrategy() = default;

  virtual const char *GetName() const = 0;
  virtual void Reset() = 0;
  virtual const ControlInput &GetCommandedInput() const = 0;
  virtual bool SetCommandedInput(ControlAxis axis, double value) = 0;
  virtual bool Update(const sim::Aircraft &aircraft, double dt,
      ControlInput &output) = 0;

  virtual bool AdjustCommandedInput(ControlAxis axis, double delta) {
    return SetCommandedInput(axis,
        GetControlAxisValue(GetCommandedInput(), axis) + delta);
  }
};
} // namespace control
