#pragma once

namespace control {
enum class FlightControlMode {
  Manual,
  Autopilot,
};

inline const char *ToString(FlightControlMode mode) {
  switch (mode) {
  case FlightControlMode::Manual:
    return "Manual";
  case FlightControlMode::Autopilot:
    return "Autopilot";
  }

  return "Unknown";
}
} // namespace control
