#include "application/gui/windows/viz/FlightVizWindow.hpp"

#include "flightui/FlightUI.hpp"
#include "application/gui/GUI.hpp"
#include "application/gui/viz/FlightVisualizer.hpp"
#include "application/sim/Simulation.hpp"

namespace gui {
namespace UI = FlightUI;

FlightVizWindow::FlightVizWindow() : Window("Flight Viz") {}

void FlightVizWindow::OnRender(gui::GUI &gui) {
  viz::FlightVisualizer *visualizer = gui.GetFlightVisualizer();
  if (visualizer == nullptr) {
    UI::TextDisabled("Flight visualization is unavailable.").Render();
    return;
  }

  visualizer->Tick(gui.GetSimulation().GetAircraft());
  visualizer->RenderScene();
}
} // namespace gui
