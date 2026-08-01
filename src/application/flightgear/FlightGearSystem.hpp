#pragma once

#include "application/flightgear/FlightGearSender.hpp"

#include <memory>

namespace sim {
class Aircraft;
}

namespace flightgear {
class FlightGearSystem {
public:
  bool Initialize();
  void Update(const sim::Aircraft &aircraft);
  void Shutdown();

private:
  std::unique_ptr<FlightGearSender> sender_;
};
} // namespace flightgear
