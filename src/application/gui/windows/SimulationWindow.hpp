#pragma once

#include "application/gui/Window.hpp"
#include "application/sim/InitialCondition.hpp"

namespace FlightUI {
class UIElement;
}

namespace gui {
class SimulationWindow final : public gui::Window {
public:
  SimulationWindow();

protected:
  void OnRender(gui::GUI &gui) override;

private:
  // Tab rendering
  void DrawInitialConditionTab(gui::GUI &gui);
  void DrawDiagnosticsTab(gui::GUI &gui);
  void DrawEnvironmentTab();
  void DrawAircraftTab(gui::GUI &gui);

  // Initial-condition controls
  FlightUI::UIElement DrawInitialConditionFields();
  FlightUI::UIElement DrawInitialConditionActions(gui::GUI &gui);

  // Simulation diagnostics
  FlightUI::UIElement DrawLastError(gui::GUI &gui) const;

  // Editable initial-condition form
  sim::InitialCondition initialCondition_;
  bool initialConditionLoaded_ = false;
};
} // namespace gui
