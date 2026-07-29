#pragma once

namespace state {
struct AircraftState {
  double simTimeSec = 0.0;

  double latitudeRad = 0.0;
  double longitudeRad = 0.0;
  double altitudeM = 0.0;

  double rollRad = 0.0;
  double pitchRad = 0.0;
  double headingRad = 0.0;
  double courseRad = 0.0;

  double uMps = 0.0;
  double vMps = 0.0;
  double wMps = 0.0;

  double pRadPerSec = 0.0;
  double qRadPerSec = 0.0;
  double rRadPerSec = 0.0;

  double airspeedMps = 0.0;
};
} // namespace state
