#include "FlightDataMonitorWindow.hpp"
#include "flightui/controls/Custom.hpp"
#include "flightui/controls/ValueLabel.hpp"
#include "flightui/layout/FoldOut.hpp"
#include "flightui/layout/HorizontalLayout.hpp"
#include "flightui/layout/VerticalLayout.hpp"
#include "flightui/plot/Plot.hpp"
#include "gui/GUI.hpp"
#include "simulation/FlightDynamics.hpp"
#include "imgui.h"
#include "simulation/Simulation.hpp"

#include <cstddef>

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr float PlotHeight = 245.0f;
constexpr float ValueSpacing = 24.0f;
constexpr std::size_t VisiblePlotSampleCount = 300;
constexpr ImPlotFlags FixedPlotFlags =
    ImPlotFlags_NoInputs | ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect
    | ImPlotFlags_NoTitle;
constexpr ImPlotAxisFlags FocusedYAxisFlags =
    ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
constexpr ImGuiTableFlags PlotTableFlags = ImGuiTableFlags_SizingStretchSame
                                           | ImGuiTableFlags_NoSavedSettings
                                           | ImGuiTableFlags_PadOuterX;
constexpr ImGuiTreeNodeFlags PlotFoldOutFlags =
    ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_SpanAvailWidth;

struct TimeRange {
  double Min = 0.0;
  double Max = 1.0;
};

TimeRange GetFocusedTimeRange(const ds::RingBuffer<double> &timeHistory) {
  if (timeHistory.empty()) {
    return {};
  }

  const double max = timeHistory[timeHistory.size() - 1];
  if (timeHistory.size() == 1) {
    return {max - 1.0, max};
  }

  const double min = timeHistory[0];
  if (min < max) {
    return {min, max};
  }

  return {max - 1.0, max + 1.0};
}
} // namespace

FlightDataMonitorWindow::FlightDataMonitorWindow()
    : gui::Window("Flight Data Monitor"), timeHistory_(VisiblePlotSampleCount),
      alphaDegHistory_(VisiblePlotSampleCount),
      betaDegHistory_(VisiblePlotSampleCount),
      pDegPerSecHistory_(VisiblePlotSampleCount),
      qDegPerSecHistory_(VisiblePlotSampleCount),
      rDegPerSecHistory_(VisiblePlotSampleCount),
      uDotMps2History_(VisiblePlotSampleCount),
      vDotMps2History_(VisiblePlotSampleCount),
      wDotMps2History_(VisiblePlotSampleCount),
      pDotDegPerSec2History_(VisiblePlotSampleCount),
      qDotDegPerSec2History_(VisiblePlotSampleCount),
      rDotDegPerSec2History_(VisiblePlotSampleCount) {}

void FlightDataMonitorWindow::OnUpdate(gui::GUI &gui) {
  auto &sim = gui.GetSimulation();
  OnRecordSamples(sim);
  DrawWindow(sim);
}

UI::UIElement FlightDataMonitorWindow::DrawAerodynamicAnglesPlot() const {
  const int offset = timeHistory_.offset();
  const TimeRange timeRange = GetFocusedTimeRange(timeHistory_);

  return UI::Plot("Aerodynamic Angles")
      .Height(PlotHeight)
      .Flags(FixedPlotFlags)
      .XAxisLimits(timeRange.Min, timeRange.Max, ImPlotCond_Always)
      .YAxisFlags(FocusedYAxisFlags)
      .XAxisLabel("Time (s)")
      .YAxisLabel("deg")
      .AddLine("alpha",
          timeHistory_.data_view(),
          alphaDegHistory_.data_view(),
          offset)
      .AddLine("beta",
          timeHistory_.data_view(),
          betaDegHistory_.data_view(),
          offset);
}

UI::UIElement FlightDataMonitorWindow::DrawBodyRatesPlot() const {
  const int offset = timeHistory_.offset();
  const TimeRange timeRange = GetFocusedTimeRange(timeHistory_);

  return UI::Plot("Body Rates")
      .Height(PlotHeight)
      .Flags(FixedPlotFlags)
      .XAxisLimits(timeRange.Min, timeRange.Max, ImPlotCond_Always)
      .YAxisFlags(FocusedYAxisFlags)
      .XAxisLabel("Time (s)")
      .YAxisLabel("deg/s")
      .AddLine("p",
          timeHistory_.data_view(),
          pDegPerSecHistory_.data_view(),
          offset)
      .AddLine("q",
          timeHistory_.data_view(),
          qDegPerSecHistory_.data_view(),
          offset)
      .AddLine("r",
          timeHistory_.data_view(),
          rDegPerSecHistory_.data_view(),
          offset);
}

UI::UIElement FlightDataMonitorWindow::DrawBodyAccelerationsPlot() const {
  const int offset = timeHistory_.offset();
  const TimeRange timeRange = GetFocusedTimeRange(timeHistory_);

  return UI::Plot("Body Accelerations")
      .Height(PlotHeight)
      .Flags(FixedPlotFlags)
      .XAxisLimits(timeRange.Min, timeRange.Max, ImPlotCond_Always)
      .YAxisFlags(FocusedYAxisFlags)
      .XAxisLabel("Time (s)")
      .YAxisLabel("m/s^2")
      .AddLine("u_dot",
          timeHistory_.data_view(),
          uDotMps2History_.data_view(),
          offset)
      .AddLine("v_dot",
          timeHistory_.data_view(),
          vDotMps2History_.data_view(),
          offset)
      .AddLine("w_dot",
          timeHistory_.data_view(),
          wDotMps2History_.data_view(),
          offset);
}

UI::UIElement FlightDataMonitorWindow::DrawAngularAccelerationsPlot() const {
  const int offset = timeHistory_.offset();
  const TimeRange timeRange = GetFocusedTimeRange(timeHistory_);

  return UI::Plot("Angular Accelerations")
      .Height(PlotHeight)
      .Flags(FixedPlotFlags)
      .XAxisLimits(timeRange.Min, timeRange.Max, ImPlotCond_Always)
      .YAxisFlags(FocusedYAxisFlags)
      .XAxisLabel("Time (s)")
      .YAxisLabel("deg/s^2")
      .AddLine("p_dot",
          timeHistory_.data_view(),
          pDotDegPerSec2History_.data_view(),
          offset)
      .AddLine("q_dot",
          timeHistory_.data_view(),
          qDotDegPerSec2History_.data_view(),
          offset)
      .AddLine("r_dot",
          timeHistory_.data_view(),
          rDotDegPerSec2History_.data_view(),
          offset);
}

void FlightDataMonitorWindow::DrawCurrentValues(
    const JSBSim::FlightProperties &prop) const {
  UI::HorizontalLayout()
      .Spacing(ValueSpacing)
      [
          +UI::ValueLabel(
              "Sim Time",
              prop.GetSimTimeSec(),
              "{:.2f}"
          )
          +UI::ValueLabel(
              "u (m/s)",
              prop.GetUMps(),
              "{:.2f}"
          )
          +UI::ValueLabel(
              "v (m/s)",
              prop.GetVMps(),
              "{:.2f}"
          )
          +UI::ValueLabel(
              "w (m/s)",
              prop.GetWMps(),
              "{:.2f}"
          )
      ]
      .Render();
}

void FlightDataMonitorWindow::DrawWindow(sim::Simulation &sim) const {
  const auto &prop = sim.GetFlightDynamics().GetProperties();

  UI::VerticalLayout()
      .Spacing(8.0f)
          [+UI::Custom([this, &prop] { DrawCurrentValues(prop); })
              + UI::VerticalLayout()
                  [+UI::FoldOut("Aerodynamic Angles")
                          .DefaultOpen()
                          .Flags(PlotFoldOutFlags)[DrawAerodynamicAnglesPlot()]
                      + UI::FoldOut("Body Rates")
                          .DefaultOpen()
                          .Flags(PlotFoldOutFlags)[DrawBodyRatesPlot()]
                      + UI::FoldOut("Body Accelerations")
                          .DefaultOpen()
                          .Flags(PlotFoldOutFlags)[DrawBodyAccelerationsPlot()]

                      + UI::FoldOut("Angular Accelerations")
                          .DefaultOpen()
                          .Flags(PlotFoldOutFlags)
                              [DrawAngularAccelerationsPlot()]]]
      .Render();
}

void FlightDataMonitorWindow::OnRecordSamples(sim::Simulation &sim) {
  const auto &prop = sim.GetFlightDynamics().GetProperties();

  timeHistory_.push_back(prop.GetSimTimeSec());
  alphaDegHistory_.push_back(prop.GetAlphaDeg());
  betaDegHistory_.push_back(prop.GetBetaDeg());
  pDegPerSecHistory_.push_back(prop.GetPDegPerSec());
  qDegPerSecHistory_.push_back(prop.GetQDegPerSec());
  rDegPerSecHistory_.push_back(prop.GetRDegPerSec());
  uDotMps2History_.push_back(prop.GetUDotMps2());
  vDotMps2History_.push_back(prop.GetVDotMps2());
  wDotMps2History_.push_back(prop.GetWDotMps2());
  pDotDegPerSec2History_.push_back(prop.GetPdotDegPerSec2());
  qDotDegPerSec2History_.push_back(prop.GetQdotDegPerSec2());
  rDotDegPerSec2History_.push_back(prop.GetRdotDegPerSec2());
}

} // namespace gui
