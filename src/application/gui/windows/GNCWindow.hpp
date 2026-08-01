#pragma once

#include "application/gui/panels/AutopilotPanel.hpp"
#include "application/gui/Window.hpp"
#include "application/sim/gnc/TrimTypes.hpp"

namespace sim {
class Aircraft;
} // namespace sim

namespace gui {
class GNCWindow final : public gui::Window {
public:
  GNCWindow();

protected:
  void OnRender(gui::GUI &gui) override;

private:
  enum class PendingTrimCommand {
    None,
    RunInitialCondition,
    CurrentState,
  };

  // Trim command handling
  void RequestTrim(PendingTrimCommand command);
  void ExecutePendingTrim(gui::GUI &gui);

  // Trim state
  gnc::TrimRequest trimRequest_;
  bool trimResultOpen_ = true;
  bool trimResidualOpen_ = true;
  bool trimInProgress_ = false;
  PendingTrimCommand pendingTrimCommand_ = PendingTrimCommand::None;

  // Autopilot UI state
  AutopilotPanelState autopilotPanelState_;
};
} // namespace gui
