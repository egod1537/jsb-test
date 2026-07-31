#pragma once

#include "application/sim/control/ControlInput.hpp"

namespace sim {
class Aircraft;
} // namespace sim

namespace control {
class FlightControlController {
public:
  virtual ~FlightControlController() = default;

  virtual const char *GetName() const = 0;
  virtual void Reset() = 0;
  virtual ControlInput Update(const sim::Aircraft &aircraft, double dt) = 0;
};
} // namespace control
