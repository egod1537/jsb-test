#include "application/gui/viz/components/TelemetryOverlay.hpp"

#include "application/gui/viz/render/LineCanvas.hpp"

#include <cstdio>

namespace viz {
void TelemetryOverlay::Render(RenderContext &context) const {
  if (!context.snapshot.viewOptions.showTelemetry) {
    return;
  }

  const auto &aircraftState = context.snapshot.aircraftState;
  const auto &controlInput = context.snapshot.controlInput;
  const char *viewMode =
      context.snapshot.viewMode == ViewMode::ThirdPerson ? "Third Person"
                                                         : "Orbit";
  const ImVec2 min = context.canvas.GetMin();
  ImDrawList &drawList = context.canvas.GetDrawList();

  char line[160]{};
  std::snprintf(line,
      sizeof(line),
      "t %.2f  View %s",
      aircraftState.simulationTimeSec,
      viewMode);
  drawList.AddText(ImVec2(min.x + 10.0F, min.y + 10.0F),
      IM_COL32(232, 238, 246, 255),
      line);

  std::snprintf(line,
      sizeof(line),
      "Alt AGL %.0f ft  CAS %.1f kt  TAS %.1f m/s",
      aircraftState.altitudeAglFt,
      aircraftState.calibratedAirspeedKts,
      aircraftState.trueAirspeedMps);
  drawList.AddText(ImVec2(min.x + 10.0F, min.y + 30.0F),
      IM_COL32(232, 238, 246, 255),
      line);

  std::snprintf(line,
      sizeof(line),
      "Roll %.1f  Pitch %.1f  Heading %.1f",
      aircraftState.rollDeg,
      aircraftState.pitchDeg,
      aircraftState.headingDeg);
  drawList.AddText(ImVec2(min.x + 10.0F, min.y + 50.0F),
      IM_COL32(232, 238, 246, 255),
      line);

  std::snprintf(line,
      sizeof(line),
      "Ail %.2f  Ele %.2f  Rud %.2f  Thr %.2f  Trim %.2f",
      controlInput.aileron,
      controlInput.elevator,
      controlInput.rudder,
      controlInput.throttle,
      context.snapshot.pitchTrim);
  drawList.AddText(ImVec2(min.x + 10.0F, min.y + 70.0F),
      IM_COL32(178, 189, 202, 255),
      line);
}
} // namespace viz
