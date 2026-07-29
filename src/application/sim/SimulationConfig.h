#pragma once
#include <string>

namespace sim {
struct SimulationConfig {
  std::string aircraftName = "c172x";

  double altitudeFt = 1000.0;
  double calibratedAirspeedKts = 80.0;

  double rollDeg = 0.0;
  double pitchDeg = 0.0;
  double headingDeg = 0.0;

  double simulationHz = 120.0;

  double GetDT() const { return 1.0 / simulationHz; }
};
} // namespace sim
