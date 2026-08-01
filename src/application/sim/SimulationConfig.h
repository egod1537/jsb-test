#pragma once
#include <string>

namespace sim {
struct SimulationConfig {
  std::string aircraftName = "c172x";

  double simulationHz = 120.0;

  double GetDT() const { return 1.0 / simulationHz; }
};
} // namespace sim
