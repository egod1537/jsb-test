#pragma once

#include "gui/Window.hpp"
#include "simulation/InitialCondition.hpp"

namespace FlightUI {
class UIElement;
}

namespace sim {
class Simulation;
}

namespace gui {
class SimulationWindow final : public gui::Window {
public:
  SimulationWindow();

protected:
  void OnUpdate(gui::GUI &gui) override;

private:
  void DrawInitialConditionTab(gui::GUI &gui);
  void DrawRuntimeTab(gui::GUI &gui);
  void DrawEnvironmentTab();
  void DrawAircraftTab(gui::GUI &gui);
  FlightUI::UIElement DrawInitialConditionFields();
  FlightUI::UIElement DrawInitialConditionActions(gui::GUI &gui);
  FlightUI::UIElement BuildRuntimeActions(sim::Simulation &simulation);
  FlightUI::UIElement DrawLastError(gui::GUI &gui) const;

  sim::InitialCondition initialCondition_;
  bool initialConditionLoaded_ = false;
};
} // namespace gui
