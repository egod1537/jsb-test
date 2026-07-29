#pragma once

namespace gui {
struct AutopilotPanelState {
  bool rollHold = false;
  bool pitchHold = false;
  bool yawHold = false;
  bool altitudeHold = false;
  bool courseHold = false;

  double rollTargetDeg = 0.0;
  double pitchTargetDeg = 0.0;
  double yawTargetDeg = 0.0;
  double altitudeTargetFt = 1000.0;
  double courseTargetDeg = 0.0;

  double rollHoldKp = 0.5;
  double rollHoldKd = 2.0;
  bool rollHoldGainsOpen = true;
};

class AutopilotPanel {
public:
  static void Draw(AutopilotPanelState &state);
};
} // namespace gui
