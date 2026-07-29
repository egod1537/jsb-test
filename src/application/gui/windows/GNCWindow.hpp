#pragma once

#include "application/gui/panels/AutopilotPanel.hpp"
#include "application/gui/Window.hpp"
#include "application/sim/gnc/TrimSolver.hpp"
#include "application/sim/gnc/TrimTypes.hpp"

namespace sim {
class Aircraft;
class Simulation;
} // namespace sim

namespace gui {
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
