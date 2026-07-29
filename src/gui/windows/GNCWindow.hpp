#pragma once

#include "gnc/TrimSolver.hpp"
#include "gnc/TrimTypes.hpp"
#include "gui/Window.hpp"

namespace sim {
class Aircraft;
class Simulation;
} // namespace sim

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
};

class GNCWindow final : public gui::Window {
public:
  GNCWindow();

protected:
  void OnUpdate(gui::GUI &gui) override;

private:
  enum class PendingTrimCommand {
    None,
    RunInitialCondition,
    CurrentState,
  };

  void RequestTrim(PendingTrimCommand command);
  void ExecutePendingTrim(sim::Simulation &simulation);

  gnc::TrimSolver trimSolver_;
  gnc::TrimRequest trimRequest_;
  gnc::TrimResult trimResult_;
  bool trimHasResult_ = false;
  bool trimResultOpen_ = true;
  bool trimResidualOpen_ = true;
  bool trimInProgress_ = false;
  PendingTrimCommand pendingTrimCommand_ = PendingTrimCommand::None;
  AutopilotPanelState autopilotPanelState_;
};
} // namespace gui
