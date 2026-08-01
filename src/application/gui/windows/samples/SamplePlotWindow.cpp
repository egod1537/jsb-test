#include "application/gui/windows/samples/SamplePlotWindow.hpp"
#include "flightui/FlightUI.hpp"
#include "application/gui/GUI.hpp"
#include <array>
#include <cmath>

namespace gui {
namespace UI = FlightUI;

SamplePlotWindow::SamplePlotWindow() : Window("ImPlot Test") {}

void SamplePlotWindow::OnRender(GUI &) {
  constexpr int POINT_COUNT = 240;

  std::array<double, POINT_COUNT> xs{};
  std::array<double, POINT_COUNT> ys{};

  const double time = UI::GetTime();
  for (int index = 0; index < POINT_COUNT; ++index) {
    xs[index] = static_cast<double>(index) / 24.0;
    ys[index] = std::sin(xs[index] + time);
  }

  UI::UIElement plot = UI::Plot("Sample Signal")
                           .Height(300.0F)
                           .XAxisLabel("Time")
                           .YAxisLabel("Value")
                           .AddLine("sin(t)",
                               FlightUI::DataView(xs.data(), xs.size()),
                               FlightUI::DataView(ys.data(), ys.size()));

  plot.Render();
}
} // namespace gui
