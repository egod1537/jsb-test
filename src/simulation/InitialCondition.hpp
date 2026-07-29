#pragma once

namespace sim {
struct InitialCondition {
  double latitudeDeg = 0.0;
  double longitudeDeg = 0.0;
  double altitudeFt = 1000.0;

  double rollDeg = 0.0;
  double pitchDeg = 0.0;
  double headingDeg = 0.0;

  double airspeedKts = 80.0;

  double pRadPerSec = 0.0;
  double qRadPerSec = 0.0;
  double rRadPerSec = 0.0;
};
} // namespace sim
