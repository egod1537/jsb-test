#include "GNCWindow.hpp"
#include "flightui/controls/Button.hpp"
#include "flightui/controls/Custom.hpp"
#include "flightui/core/UIElement.hpp"
#include "flightui/layout/FoldOut.hpp"
#include "flightui/layout/TabGroup.hpp"
#include "flightui/layout/VerticalLayout.hpp"
#include "gui/GUI.hpp"
#include "simulation/FlightDynamics.hpp"

#include <cstddef>
#include <cstdio>
#include <iterator>
#include <imgui.h>

namespace gui {
namespace UI = FlightUI;

namespace {
struct TrimValueMetric {
  const char *Label;
  double Value;
  const char *Format;
};

constexpr ImGuiTableFlags MetricTableFlags = ImGuiTableFlags_SizingStretchProp
                                             | ImGuiTableFlags_NoSavedSettings
                                             | ImGuiTableFlags_PadOuterX;

constexpr float TrimInputPanelHeight = 178.0F;
constexpr float FoldOutSpacing = 6.0F;
constexpr ImGuiTreeNodeFlags FoldOutFlags = ImGuiTreeNodeFlags_Framed
                                            | ImGuiTreeNodeFlags_SpanAvailWidth
                                            | ImGuiTreeNodeFlags_DefaultOpen;
constexpr float TrimButtonWidth = 160.0F;

const char *TrimModeLabel(gnc::TrimMode mode) {
  switch (mode) {
  case gnc::TrimMode::Longitudinal:
    return "Longitudinal";
  case gnc::TrimMode::Full:
    return "Full";
  case gnc::TrimMode::Ground:
    return "Ground";
  }

  return "Unknown";
}

int TrimModeIndex(gnc::TrimMode mode) {
  switch (mode) {
  case gnc::TrimMode::Longitudinal:
    return 0;
  case gnc::TrimMode::Full:
    return 1;
  case gnc::TrimMode::Ground:
    return 2;
  }

  return 0;
}

gnc::TrimMode TrimModeFromIndex(int index) {
  switch (index) {
  case 1:
    return gnc::TrimMode::Full;
  case 2:
    return gnc::TrimMode::Ground;
  case 0:
  default:
    return gnc::TrimMode::Longitudinal;
  }
}

void DrawSectionTitle(const char *title) {
#if IMGUI_VERSION_NUM >= 18900
  ImGui::SeparatorText(title);
#else
  ImGui::TextUnformatted(title);
  ImGui::Separator();
#endif
}

void DrawMetricGrid(const char *id, const TrimValueMetric *metrics,
    std::size_t metricCount, int metricsPerRow) {
  const int tableColumnCount = metricsPerRow * 2;
  if (!ImGui::BeginTable(id, tableColumnCount, MetricTableFlags)) {
    return;
  }

  for (int column = 0; column < tableColumnCount; ++column) {
    const bool isLabelColumn = column % 2 == 0;
    ImGui::TableSetupColumn(nullptr,
        isLabelColumn ? ImGuiTableColumnFlags_WidthFixed
                      : ImGuiTableColumnFlags_WidthStretch,
        isLabelColumn ? 120.0F : 1.0F);
  }

  for (std::size_t index = 0; index < metricCount; ++index) {
    if (index % static_cast<std::size_t>(metricsPerRow) == 0) {
      ImGui::TableNextRow();
    }

    ImGui::TableNextColumn();
    ImGui::TextDisabled("%s", metrics[index].Label);
    ImGui::TableNextColumn();

    char valueText[64] = {};
    std::snprintf(valueText,
        sizeof(valueText),
        metrics[index].Format,
        metrics[index].Value);
    ImGui::TextUnformatted(valueText);
  }

  ImGui::EndTable();
}

void DrawTrimInputPanel(gnc::TrimRequest &request) {
  if (ImGui::BeginChild("TrimInputPanel",
          ImVec2(0.0F, TrimInputPanelHeight),
          ImGuiChildFlags_Borders)) {
    DrawSectionTitle("Trim Input");

    int modeIndex = TrimModeIndex(request.mode);
    constexpr const char *modeLabels[] = {"Longitudinal", "Full", "Ground"};
    if (ImGui::Combo("Mode", &modeIndex, modeLabels, std::size(modeLabels))) {
      request.mode = TrimModeFromIndex(modeIndex);
    }

    ImGui::InputDouble("Airspeed (kt)",
        &request.airspeedKts,
        1.0,
        10.0,
        "%.2f");
    ImGui::InputDouble("Altitude (ft)",
        &request.altitudeFt,
        100.0,
        1000.0,
        "%.2f");
    ImGui::InputDouble("Flight Path Angle (deg)",
        &request.flightPathAngleDeg,
        0.1,
        1.0,
        "%.2f");
  }
  ImGui::EndChild();
}

void DrawTrimRequestSummary(const gnc::TrimRequest &request) {
  if (!ImGui::BeginTable("TrimRequestSummaryTable", 8, MetricTableFlags)) {
    return;
  }

  for (int column = 0; column < 8; ++column) {
    const bool isLabelColumn = column % 2 == 0;
    ImGui::TableSetupColumn(nullptr,
        isLabelColumn ? ImGuiTableColumnFlags_WidthFixed
                      : ImGuiTableColumnFlags_WidthStretch,
        isLabelColumn ? 120.0F : 1.0F);
  }

  ImGui::TableNextRow();
  ImGui::TableNextColumn();
  ImGui::TextDisabled("Mode");
  ImGui::TableNextColumn();
  ImGui::TextUnformatted(TrimModeLabel(request.mode));
  ImGui::TableNextColumn();
  ImGui::TextDisabled("Airspeed");
  ImGui::TableNextColumn();
  ImGui::Text("%.2f kt", request.airspeedKts);
  ImGui::TableNextColumn();
  ImGui::TextDisabled("Altitude");
  ImGui::TableNextColumn();
  ImGui::Text("%.2f ft", request.altitudeFt);
  ImGui::TableNextColumn();
  ImGui::TextDisabled("Flight Path Angle");
  ImGui::TableNextColumn();
  ImGui::Text("%.2f deg", request.flightPathAngleDeg);

  ImGui::EndTable();
}

void DrawTrimResultContent(const gnc::TrimResult &result, bool hasResult) {
  ImGui::TextDisabled("Status");
  ImGui::SameLine();
  if (!hasResult) {
    ImGui::TextUnformatted("Idle");
  } else {
    ImGui::TextUnformatted(result.success ? "Success" : "Failed");
  }

  if (!result.message.empty()) {
    ImGui::TextDisabled("Message");
    ImGui::SameLine();
    ImGui::TextWrapped("%s", result.message.c_str());
  }

  const TrimValueMetric metrics[] = {
      {"Alpha", result.alphaDeg, "%.2f deg"},
      {"Beta", result.betaDeg, "%.2f deg"},
      {"Roll", result.rollDeg, "%.2f deg"},
      {"Pitch", result.pitchDeg, "%.2f deg"},
      {"Throttle", result.throttle, "%.3f"},
      {"Elevator", result.elevator, "%.3f"},
      {"Pitch Trim", result.pitchTrim, "%.3f"},
      {"Aileron", result.aileron, "%.3f"},
      {"Rudder", result.rudder, "%.3f"},
  };

  DrawMetricGrid("TrimResultMetrics", metrics, std::size(metrics), 2);
}

void DrawTrimResidualContent(const gnc::TrimResult &result) {
  const TrimValueMetric metrics[] = {
      {"uDot", result.uDot, "%.4f m/s^2"},
      {"vDot", result.vDot, "%.4f m/s^2"},
      {"wDot", result.wDot, "%.4f m/s^2"},
      {"pDot", result.pDot, "%.4f deg/s^2"},
      {"qDot", result.qDot, "%.4f deg/s^2"},
      {"rDot", result.rDot, "%.4f deg/s^2"},
  };

  DrawMetricGrid("TrimResidualMetrics", metrics, std::size(metrics), 2);
}

void DrawTrimTab(gnc::TrimRequest &request, gnc::TrimResult &result,
    bool &hasResult, bool &resultOpen, bool &residualOpen,
    sim::FlightDynamics &flightDynamics) {
  DrawTrimInputPanel(request);
  ImGui::Spacing();

  const UI::UIElement trimFromInputButton =
      UI::Button("RunIC Trim").Width(TrimButtonWidth).OnAction([&] {
        result = flightDynamics.Trim(request);
        hasResult = true;
        resultOpen = true;
        residualOpen = true;
      });
  trimFromInputButton.Render();

  ImGui::SameLine();
  const UI::UIElement trimCurrentStateButton =
      UI::Button("Current State Trim").Width(TrimButtonWidth).OnAction([&] {
        result = flightDynamics.TrimCurrentState(request.mode);
        hasResult = true;
        resultOpen = true;
        residualOpen = true;
      });
  trimCurrentStateButton.Render();

  ImGui::Spacing();
  DrawTrimRequestSummary(request);
  ImGui::Dummy(ImVec2(0.0F, FoldOutSpacing));

  UI::FoldOut("Result")
      .Open(resultOpen)
      .Flags(FoldOutFlags)[UI::Custom(
          [&result, hasResult] { DrawTrimResultContent(result, hasResult); })]
      .Render();

  UI::FoldOut("Residual")
      .Open(residualOpen)
      .Flags(FoldOutFlags)[UI::Custom(
          [&result] { DrawTrimResidualContent(result); })]
      .Render();
}
} // namespace

GNCWindow::GNCWindow() : Window("GNC") {}

void GNCWindow::OnUpdate(gui::GUI &gui) {
  auto &flightDynamics = gui.GetSimulation().GetFlightDynamics();

  UI::VerticalLayout()[+UI::TabGroup("GNC")[+UI::Tab(
                           "Trim")[UI::Custom([this, &flightDynamics] {
    DrawTrimTab(trimRequest_,
        trimResult_,
        trimHasResult_,
        trimResultOpen_,
        trimResidualOpen_,
        flightDynamics);
  })]]]
      .Render();
}
} // namespace gui
