#include "FlightDataMonitorWindow.hpp"
#include "flightui/controls/Custom.hpp"
#include "flightui/controls/ValueLabel.hpp"
#include "flightui/layout/FoldOut.hpp"
#include "flightui/layout/HorizontalLayout.hpp"
#include "flightui/layout/VerticalLayout.hpp"
#include "flightui/plot/Plot.hpp"
#include "gui/GUI.hpp"
#include "simulation/Aircraft.hpp"
#include "simulation/AircraftState.hpp"
#include "simulation/Simulation.hpp"

#include <cstddef>

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr float PlotHeight = 245.0f;
constexpr float ValueSpacing = 24.0f;
constexpr std::size_t VisiblePlotSampleCount = 300;

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
    : gui::Window("Monitor"), timeHistory_(VisiblePlotSampleCount),
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
      .FixedView()
      .XAxisLimitsAlways(timeRange.Min, timeRange.Max)
      .FocusedYAxis()
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
      .FixedView()
      .XAxisLimitsAlways(timeRange.Min, timeRange.Max)
      .FocusedYAxis()
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
      .FixedView()
      .XAxisLimitsAlways(timeRange.Min, timeRange.Max)
      .FocusedYAxis()
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
      .FixedView()
      .XAxisLimitsAlways(timeRange.Min, timeRange.Max)
      .FocusedYAxis()
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
    const sim::AircraftState &state) const {
  UI::HorizontalLayout()
      .Spacing(ValueSpacing)
          [+UI::ValueLabel("Sim Time", state.simulationTimeSec, "{:.2f}")
              + UI::ValueLabel("u (m/s)", state.uMps, "{:.2f}")
              + UI::ValueLabel("v (m/s)", state.vMps, "{:.2f}")
              + UI::ValueLabel("w (m/s)", state.wMps, "{:.2f}")]
      .Render();
}

void FlightDataMonitorWindow::DrawWindow(sim::Simulation &sim) const {
  const sim::AircraftState aircraftState =
      sim.GetAircraft().GetAircraftState();

  UI::VerticalLayout()
      .Spacing(
          8.0f)[+UI::Custom(
                    [this, aircraftState] { DrawCurrentValues(aircraftState); })
                + UI::VerticalLayout()
                    [+UI::FoldOut("Aerodynamic Angles")
                            .DefaultOpen()
                            .Framed()
                            .SpanAvailWidth()[DrawAerodynamicAnglesPlot()]
                        + UI::FoldOut("Body Rates")
                            .DefaultOpen()
                            .Framed()
                            .SpanAvailWidth()[DrawBodyRatesPlot()]
                        + UI::FoldOut("Body Accelerations")
                            .DefaultOpen()
                            .Framed()
                            .SpanAvailWidth()[DrawBodyAccelerationsPlot()]

                        + UI::FoldOut("Angular Accelerations")
                            .DefaultOpen()
                            .Framed()
                            .SpanAvailWidth()[DrawAngularAccelerationsPlot()]]]
      .Render();
}

void FlightDataMonitorWindow::OnRecordSamples(sim::Simulation &sim) {
  const sim::AircraftState state = sim.GetAircraft().GetAircraftState();
  const sim::AircraftStateDerivative derivative =
      sim.GetAircraft().GetAircraftStateDerivative();

  timeHistory_.push_back(state.simulationTimeSec);
  alphaDegHistory_.push_back(state.alphaDeg);
  betaDegHistory_.push_back(state.betaDeg);
  pDegPerSecHistory_.push_back(state.pDegPerSec);
  qDegPerSecHistory_.push_back(state.qDegPerSec);
  rDegPerSecHistory_.push_back(state.rDegPerSec);
  uDotMps2History_.push_back(derivative.uDotMps2);
  vDotMps2History_.push_back(derivative.vDotMps2);
  wDotMps2History_.push_back(derivative.wDotMps2);
  pDotDegPerSec2History_.push_back(derivative.pDotDegPerSec2);
  qDotDegPerSec2History_.push_back(derivative.qDotDegPerSec2);
  rDotDegPerSec2History_.push_back(derivative.rDotDegPerSec2);
}

} // namespace gui
