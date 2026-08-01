#pragma once

#include <functional>

namespace gui {
struct AutopilotPanelState {
  // Hold selection
  bool rollHold = false;
  bool pitchHold = false;
  bool yawHold = false;
  bool altitudeHold = false;
  bool courseHold = false;

  // Hold targets
  double rollTargetDeg = 0.0;
  double pitchTargetDeg = 0.0;
  double yawTargetDeg = 0.0;
  double altitudeTargetFt = 1000.0;
  double courseTargetDeg = 0.0;

  // Controller tuning
  double rollHoldKp = 0.5;
  double rollHoldKd = 2.0;
  bool rollHoldGainsOpen = true;
  double pitchHoldKp = 0.5;
  double pitchHoldKd = 2.0;
  bool pitchHoldGainsOpen = true;
};

struct AutopilotPanelProps {
  AutopilotPanelState &state;

  // Roll telemetry and actions
  double currentRollDeg = 0.0;
  double currentRollRateDegPerSec = 0.0;
  double currentAileron = 0.0;
  bool rollHoldActive = false;
  std::function<void()> captureCurrentRoll;

  // Pitch telemetry and actions
  double currentPitchDeg = 0.0;
  double currentPitchRateDegPerSec = 0.0;
  double currentElevator = 0.0;
  bool pitchHoldActive = false;
  std::function<void()> captureCurrentPitch;
};

class AutopilotPanel {
public:
  static void Draw(AutopilotPanelState &state);
  static void Draw(const AutopilotPanelProps &props);
};
} // namespace gui
