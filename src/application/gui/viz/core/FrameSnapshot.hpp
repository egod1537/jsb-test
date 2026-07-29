#pragma once

#include "application/gui/viz/core/Math.hpp"
#include "application/sim/control/ControlInput.hpp"
#include "application/sim/AircraftState.hpp"

namespace viz {
enum class ViewMode {
  Orbit,
  ThirdPerson,
};

struct ViewOptions {
  bool showGroundGrid = true;
  bool showTelemetry = true;
};

struct FrameSnapshot {
  sim::AircraftState aircraftState{};
  control::ControlInput controlInput{};
  double pitchTrim = 0.0;
  ViewMode viewMode = ViewMode::Orbit;
  ViewOptions viewOptions{};
  Vec3 groundScroll{};
  float visualAltitude = 0.0F;
  bool hasAircraft = false;
};
} // namespace viz
