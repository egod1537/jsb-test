#include "FlightDataMonitorWindow.hpp"
#include "flightui/controls/Custom.hpp"
#include "flightui/controls/ValueLabel.hpp"
#include "flightui/layout/FoldOut.hpp"
#include "flightui/layout/HorizontalLayout.hpp"
#include "flightui/layout/VerticalLayout.hpp"
#include "flightui/plot/Plot.hpp"
#include "flightui/core/UIScale.hpp"
#include "application/gui/GUI.hpp"
#include "application/sim/Aircraft.hpp"
#include "application/sim/AircraftState.hpp"
#include "application/sim/Simulation.hpp"

#include <imgui.h>

#include <cstddef>
#include <utility>

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr float PlotHeight = 245.0f;
constexpr float ValueSpacing = 24.0f;
constexpr float PlotGridMinColumnWidth = 430.0f;
constexpr float PlotGridColumnSpacing = 8.0f;
constexpr float PlotSelectorButtonWidth = 96.0f;
constexpr std::size_t VisiblePlotSampleCount = 300;
constexpr double MetersPerSecondToKnots = 1.9438444924406048;

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
      rollDegHistory_(VisiblePlotSampleCount),
      pitchDegHistory_(VisiblePlotSampleCount),
      headingDegHistory_(VisiblePlotSampleCount),
      uMpsHistory_(VisiblePlotSampleCount),
      vMpsHistory_(VisiblePlotSampleCount),
      wMpsHistory_(VisiblePlotSampleCount),
      pDegPerSecHistory_(VisiblePlotSampleCount),
      qDegPerSecHistory_(VisiblePlotSampleCount),
      rDegPerSecHistory_(VisiblePlotSampleCount),
      calibratedAirspeedKtsHistory_(VisiblePlotSampleCount),
      trueAirspeedKtsHistory_(VisiblePlotSampleCount),
      altitudeAglFtHistory_(VisiblePlotSampleCount),
      uDotMps2History_(VisiblePlotSampleCount),
      vDotMps2History_(VisiblePlotSampleCount),
      wDotMps2History_(VisiblePlotSampleCount),
      pDotDegPerSec2History_(VisiblePlotSampleCount),
      qDotDegPerSec2History_(VisiblePlotSampleCount),
      rDotDegPerSec2History_(VisiblePlotSampleCount) {}

void FlightDataMonitorWindow::OnRender(gui::GUI &gui) {
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

UI::UIElement FlightDataMonitorWindow::DrawAttitudePlot() const {
  const int offset = timeHistory_.offset();
  const TimeRange timeRange = GetFocusedTimeRange(timeHistory_);

  return UI::Plot("Attitude")
      .Height(PlotHeight)
      .FixedView()
      .XAxisLimitsAlways(timeRange.Min, timeRange.Max)
      .FocusedYAxis()
      .XAxisLabel("Time (s)")
      .YAxisLabel("deg")
      .AddLine("roll",
          timeHistory_.data_view(),
          rollDegHistory_.data_view(),
          offset)
      .AddLine("pitch",
          timeHistory_.data_view(),
          pitchDegHistory_.data_view(),
          offset)
      .AddLine("heading",
          timeHistory_.data_view(),
          headingDegHistory_.data_view(),
          offset);
}

UI::UIElement FlightDataMonitorWindow::DrawBodyVelocitiesPlot() const {
  const int offset = timeHistory_.offset();
  const TimeRange timeRange = GetFocusedTimeRange(timeHistory_);

  return UI::Plot("Body Velocities")
      .Height(PlotHeight)
      .FixedView()
      .XAxisLimitsAlways(timeRange.Min, timeRange.Max)
      .FocusedYAxis()
      .XAxisLabel("Time (s)")
      .YAxisLabel("m/s")
      .AddLine("u", timeHistory_.data_view(), uMpsHistory_.data_view(), offset)
      .AddLine("v", timeHistory_.data_view(), vMpsHistory_.data_view(), offset)
      .AddLine("w", timeHistory_.data_view(), wMpsHistory_.data_view(), offset);
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

UI::UIElement FlightDataMonitorWindow::DrawAirspeedPlot() const {
  const int offset = timeHistory_.offset();
  const TimeRange timeRange = GetFocusedTimeRange(timeHistory_);

  return UI::Plot("Airspeed")
      .Height(PlotHeight)
      .FixedView()
      .XAxisLimitsAlways(timeRange.Min, timeRange.Max)
      .FocusedYAxis()
      .XAxisLabel("Time (s)")
      .YAxisLabel("kt")
      .AddLine("calibrated",
          timeHistory_.data_view(),
          calibratedAirspeedKtsHistory_.data_view(),
          offset)
      .AddLine("true",
          timeHistory_.data_view(),
          trueAirspeedKtsHistory_.data_view(),
          offset);
}

UI::UIElement FlightDataMonitorWindow::DrawAltitudePlot() const {
  const int offset = timeHistory_.offset();
  const TimeRange timeRange = GetFocusedTimeRange(timeHistory_);

  return UI::Plot("Altitude AGL")
      .Height(PlotHeight)
      .FixedView()
      .XAxisLimitsAlways(timeRange.Min, timeRange.Max)
      .FocusedYAxis()
      .XAxisLabel("Time (s)")
      .YAxisLabel("ft")
      .AddLine("altitude",
          timeHistory_.data_view(),
          altitudeAglFtHistory_.data_view(),
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
      .Spacing(ValueSpacing)[+UI::ValueLabel("Sim Time",
                                 state.simulationTimeSec,
                                 "{:.2f}")
                             + UI::ValueLabel("u (m/s)", state.uMps, "{:.2f}")
                             + UI::ValueLabel("v (m/s)", state.vMps, "{:.2f}")
                             + UI::ValueLabel("w (m/s)", state.wMps, "{:.2f}")]
      .Render();
}

void FlightDataMonitorWindow::DrawPlotSelector() {
  ImGui::PushID("MonitorPlotSelector");

  if (ImGui::Button("Plots", ImVec2(UI::Ui(PlotSelectorButtonWidth), 0.0f))) {
    ImGui::OpenPopup("PlotOptions");
  }

  if (ImGui::BeginPopup("PlotOptions")) {
    ImGui::TextDisabled("Visible plots");
    ImGui::Separator();
    ImGui::Checkbox("Aerodynamic Angles", &plotVisibility_.aerodynamicAngles);
    ImGui::Checkbox("Attitude", &plotVisibility_.attitude);
    ImGui::Checkbox("Body Velocities", &plotVisibility_.bodyVelocities);
    ImGui::Checkbox("Body Rates", &plotVisibility_.bodyRates);
    ImGui::Checkbox("Airspeed", &plotVisibility_.airspeed);
    ImGui::Checkbox("Altitude AGL", &plotVisibility_.altitude);
    ImGui::Checkbox("Body Accelerations", &plotVisibility_.bodyAccelerations);
    ImGui::Checkbox("Angular Accelerations",
        &plotVisibility_.angularAccelerations);
    ImGui::EndPopup();
  }

  ImGui::PopID();
}

void FlightDataMonitorWindow::DrawPlotGrid() const {
  if (!plotVisibility_.aerodynamicAngles && !plotVisibility_.attitude
      && !plotVisibility_.bodyVelocities && !plotVisibility_.bodyRates
      && !plotVisibility_.airspeed && !plotVisibility_.altitude
      && !plotVisibility_.bodyAccelerations
      && !plotVisibility_.angularAccelerations) {
    ImGui::TextDisabled("No plots selected.");
    return;
  }

  const float availableWidth = ImGui::GetContentRegionAvail().x;
  const int columnCount =
      availableWidth >= UI::Ui(PlotGridMinColumnWidth * 2.0f) ? 2 : 1;
  constexpr ImGuiTableFlags Flags =
      ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_NoSavedSettings;

  if (!ImGui::BeginTable("MonitorPlotGrid", columnCount, Flags)) {
    return;
  }

  for (int columnIndex = 0; columnIndex < columnCount; ++columnIndex) {
    ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthStretch);
  }

  std::size_t plotIndex = 0;
  const auto renderPlot = [&](const char *title, UI::UIElement plot) {
    if (plotIndex % static_cast<std::size_t>(columnCount) == 0U) {
      ImGui::TableNextRow();
    }

    ImGui::TableNextColumn();
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,
        ImVec2(UI::Ui(PlotGridColumnSpacing), UI::Ui(8.0f)));
    UI::FoldOut(title)
        .DefaultOpen()
        .Framed()
        .SpanAvailWidth()[std::move(plot)]
        .Render();
    ImGui::PopStyleVar();
    ++plotIndex;
  };

  if (plotVisibility_.aerodynamicAngles) {
    renderPlot("Aerodynamic Angles", DrawAerodynamicAnglesPlot());
  }
  if (plotVisibility_.attitude) {
    renderPlot("Attitude", DrawAttitudePlot());
  }
  if (plotVisibility_.bodyVelocities) {
    renderPlot("Body Velocities", DrawBodyVelocitiesPlot());
  }
  if (plotVisibility_.bodyRates) {
    renderPlot("Body Rates", DrawBodyRatesPlot());
  }
  if (plotVisibility_.airspeed) {
    renderPlot("Airspeed", DrawAirspeedPlot());
  }
  if (plotVisibility_.altitude) {
    renderPlot("Altitude AGL", DrawAltitudePlot());
  }
  if (plotVisibility_.bodyAccelerations) {
    renderPlot("Body Accelerations", DrawBodyAccelerationsPlot());
  }
  if (plotVisibility_.angularAccelerations) {
    renderPlot("Angular Accelerations", DrawAngularAccelerationsPlot());
  }

  ImGui::EndTable();
}

void FlightDataMonitorWindow::DrawWindow(sim::Simulation &sim) {
  const sim::AircraftState aircraftState = sim.GetAircraft().GetAircraftState();

  UI::VerticalLayout()
      .Spacing(8.0f)[+UI::Custom([this, aircraftState] {
        DrawCurrentValues(aircraftState);
      }) + UI::Custom([this] { DrawPlotSelector(); })
                     + UI::Custom([this] { DrawPlotGrid(); })]
      .Render();
}

void FlightDataMonitorWindow::OnRecordSamples(sim::Simulation &sim) {
  const sim::AircraftState state = sim.GetAircraft().GetAircraftState();
  const sim::AircraftStateDerivative derivative =
      sim.GetAircraft().GetAircraftStateDerivative();

  timeHistory_.push_back(state.simulationTimeSec);
  alphaDegHistory_.push_back(state.alphaDeg);
  betaDegHistory_.push_back(state.betaDeg);
  rollDegHistory_.push_back(state.rollDeg);
  pitchDegHistory_.push_back(state.pitchDeg);
  headingDegHistory_.push_back(state.headingDeg);
  uMpsHistory_.push_back(state.uMps);
  vMpsHistory_.push_back(state.vMps);
  wMpsHistory_.push_back(state.wMps);
  pDegPerSecHistory_.push_back(state.pDegPerSec);
  qDegPerSecHistory_.push_back(state.qDegPerSec);
  rDegPerSecHistory_.push_back(state.rDegPerSec);
  calibratedAirspeedKtsHistory_.push_back(state.calibratedAirspeedKts);
  trueAirspeedKtsHistory_.push_back(
      state.trueAirspeedMps * MetersPerSecondToKnots);
  altitudeAglFtHistory_.push_back(state.altitudeAglFt);
  uDotMps2History_.push_back(derivative.uDotMps2);
  vDotMps2History_.push_back(derivative.vDotMps2);
  wDotMps2History_.push_back(derivative.wDotMps2);
  pDotDegPerSec2History_.push_back(derivative.pDotDegPerSec2);
  qDotDegPerSec2History_.push_back(derivative.qDotDegPerSec2);
  rDotDegPerSec2History_.push_back(derivative.rDotDegPerSec2);
}

} // namespace gui
