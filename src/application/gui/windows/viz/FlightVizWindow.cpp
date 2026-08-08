#include "application/gui/windows/viz/FlightVizWindow.hpp"

#include "application/gui/GUI.hpp"
#include "application/gui/viz/FlightVisualizer.hpp"
#include "application/sim/Simulation.hpp"
#include "flightui/FlightUI.hpp"

#include <array>
#include <cmath>
#include <imgui.h>
#include <string>

namespace {
constexpr float RuntimePanelHeight = 52.0F;
constexpr float RuntimeControlSpacing = 8.0F;
constexpr float TransportButtonSize = 36.0F;
constexpr float SpeedButtonWidth = 40.0F;
constexpr float ControlCornerRadius = 6.0F;
constexpr std::array<int, 4> SimulationSpeeds = {1, 2, 3, 4};

enum class TransportIcon {
  Pause,
  Step,
  Play,
  Reset,
};

ImVec2 Offset(ImVec2 point, float x, float y) {
  return {point.x + x, point.y + y};
}

void DrawTransportIcon(ImDrawList &drawList, TransportIcon icon, ImVec2 center,
    ImU32 color) {
  switch (icon) {
  case TransportIcon::Pause:
    drawList.AddRectFilled(Offset(center, -6.0F, -8.0F),
        Offset(center, -2.0F, 8.0F),
        color,
        1.0F);
    drawList.AddRectFilled(Offset(center, 2.0F, -8.0F),
        Offset(center, 6.0F, 8.0F),
        color,
        1.0F);
    break;
  case TransportIcon::Step:
    drawList.AddTriangleFilled(Offset(center, -7.0F, -8.0F),
        Offset(center, -7.0F, 8.0F),
        Offset(center, 5.0F, 0.0F),
        color);
    drawList.AddRectFilled(Offset(center, 7.0F, -8.0F),
        Offset(center, 10.0F, 8.0F),
        color,
        1.0F);
    break;
  case TransportIcon::Play:
    drawList.AddTriangleFilled(Offset(center, -6.0F, -9.0F),
        Offset(center, -6.0F, 9.0F),
        Offset(center, 9.0F, 0.0F),
        color);
    break;
  case TransportIcon::Reset:
    drawList.PathArcTo(center, 8.0F, -0.75F, 4.35F, 24);
    drawList.PathStroke(color, 0, 2.4F);
    drawList.AddTriangleFilled(Offset(center, -7.5F, -6.5F),
        Offset(center, -10.0F, -0.5F),
        Offset(center, -3.5F, -2.5F),
        color);
    break;
  }
}

bool DrawTransportButton(const char *id, TransportIcon icon, bool enabled,
    const char *tooltip) {
  ImGui::PushID(id);
  ImGui::BeginDisabled(!enabled);

  const ImVec2 minimum = ImGui::GetCursorScreenPos();
  const ImVec2 size{TransportButtonSize, TransportButtonSize};
  const bool clicked = ImGui::InvisibleButton("##Button", size);
  const bool hovered =
      ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
  const bool held = ImGui::IsItemActive();

  ImGui::EndDisabled();

  ImGuiCol backgroundColor = ImGuiCol_Button;
  if (held) {
    backgroundColor = ImGuiCol_ButtonActive;
  } else if (hovered && enabled) {
    backgroundColor = ImGuiCol_ButtonHovered;
  }

  ImDrawList *drawList = ImGui::GetWindowDrawList();
  const ImVec2 maximum = Offset(minimum, size.x, size.y);
  drawList->AddRectFilled(minimum,
      maximum,
      ImGui::GetColorU32(backgroundColor),
      ControlCornerRadius);
  drawList->AddRect(minimum,
      maximum,
      ImGui::GetColorU32(ImGuiCol_Border),
      ControlCornerRadius);
  DrawTransportIcon(*drawList,
      icon,
      Offset(minimum, size.x * 0.5F, size.y * 0.5F),
      ImGui::GetColorU32(enabled ? ImGuiCol_Text : ImGuiCol_TextDisabled));

  if (hovered && tooltip != nullptr) {
    ImGui::SetTooltip("%s", tooltip);
  }

  ImGui::PopID();
  return clicked && enabled;
}

void DrawStatusBadge(application::SimulationExecutionState state) {
  const bool isRunning =
      state == application::SimulationExecutionState::Running;
  const char *label = application::ToString(state);
  const ImVec2 textSize = ImGui::CalcTextSize(label);
  const ImVec2 size{textSize.x + 30.0F, TransportButtonSize};
  const ImVec2 minimum = ImGui::GetCursorScreenPos();
  const ImVec2 maximum = Offset(minimum, size.x, size.y);
  const ImVec4 accent = isRunning ? ImVec4(0.25F, 0.78F, 0.45F, 1.0F)
                                  : ImVec4(0.95F, 0.66F, 0.22F, 1.0F);

  ImGui::Dummy(size);

  ImDrawList *drawList = ImGui::GetWindowDrawList();
  drawList->AddRectFilled(minimum,
      maximum,
      ImGui::GetColorU32(ImVec4(accent.x, accent.y, accent.z, 0.13F)),
      ControlCornerRadius);
  drawList->AddRect(minimum,
      maximum,
      ImGui::GetColorU32(ImVec4(accent.x, accent.y, accent.z, 0.55F)),
      ControlCornerRadius);
  drawList->AddCircleFilled(Offset(minimum, 12.0F, size.y * 0.5F),
      3.5F,
      ImGui::GetColorU32(accent));
  drawList->AddText(Offset(minimum, 21.0F, (size.y - textSize.y) * 0.5F),
      ImGui::GetColorU32(ImGuiCol_Text),
      label);
}

bool DrawSpeedButton(int speed, bool selected) {
  const std::string label = std::to_string(speed) + "x";
  if (selected) {
    ImGui::PushStyleColor(ImGuiCol_Button,
        ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
    ImGui::PushStyleColor(ImGuiCol_Border,
        ImGui::GetStyleColorVec4(ImGuiCol_CheckMark));
  }

  const bool clicked =
      ImGui::Button(label.c_str(), {SpeedButtonWidth, TransportButtonSize});

  if (selected) {
    ImGui::PopStyleColor(2);
  }

  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Run at %dx speed", speed);
  }

  return clicked;
}
} // namespace

namespace gui {
namespace UI = FlightUI;

FlightVizWindow::FlightVizWindow() : Window("Flight Viz") {}

void FlightVizWindow::OnRender(gui::GUI &gui) {
  DrawRuntimePanel(gui);

  viz::FlightVisualizer *visualizer = gui.GetFlightVisualizer();
  if (visualizer == nullptr) {
    UI::TextDisabled("Flight visualization is unavailable.").Render();
    return;
  }

  visualizer->Tick(gui.GetSimulation().GetAircraft());
  visualizer->RenderScene();
}

void FlightVizWindow::DrawRuntimePanel(gui::GUI &gui) {
  auto &executionControl = gui.GetSimulationExecutionControl();
  const application::SimulationExecutionState executionState =
      executionControl.GetSimulationExecutionState();
  const bool isRunning =
      executionState == application::SimulationExecutionState::Running;
  const bool isPaused =
      executionState == application::SimulationExecutionState::Paused;

  UI::Panel("RuntimeControlPanel")
      .FlexibleWidth(true)
      .Height(RuntimePanelHeight)
      .Flags(ImGuiChildFlags_AlwaysUseWindowPadding)
      .Border(true)[UI::Custom(
          [&executionControl, executionState, isRunning, isPaused] {
            if (DrawTransportButton("PauseSimulation",
                    TransportIcon::Pause,
                    isRunning,
                    "Pause automatic execution")) {
              executionControl.PauseSimulation();
            }

            ImGui::SameLine(0.0F, RuntimeControlSpacing);
            if (DrawTransportButton("StepSimulation",
                    TransportIcon::Step,
                    isPaused,
                    "Advance exactly one 1/30-second simulation tick")) {
              executionControl.RequestSimulationTick();
            }

            ImGui::SameLine(0.0F, RuntimeControlSpacing);
            if (DrawTransportButton("RunSimulation",
                    TransportIcon::Play,
                    isPaused,
                    "Resume automatic execution")) {
              executionControl.ResumeSimulation();
            }

            ImGui::SameLine(0.0F, RuntimeControlSpacing);
            if (DrawTransportButton("ResetSimulation",
                    TransportIcon::Reset,
                    executionState
                        != application::SimulationExecutionState::Stopped,
                    "Reset simulation")) {
              const bool resumeAfterReset = isRunning;
              executionControl.PauseSimulation();
              if (executionControl.ResetSimulation() && resumeAfterReset) {
                executionControl.ResumeSimulation();
              }
            }

            ImGui::SameLine(0.0F, RuntimeControlSpacing);
            DrawStatusBadge(executionState);
            ImGui::SameLine(0.0F, RuntimeControlSpacing * 2.0F);
            ImGui::AlignTextToFramePadding();
            ImGui::TextDisabled("Speed");

            const double automaticHz =
                executionControl.GetAutomaticSimulationHz();
            for (const int speed : SimulationSpeeds) {
              ImGui::SameLine(0.0F, 4.0F);
              const double speedHz = sim::DefaultSimulationHz * speed;
              if (DrawSpeedButton(speed,
                      std::abs(automaticHz - speedHz) < 0.5)) {
                executionControl.SetAutomaticSimulationHz(speedHz);
              }
            }

            ImGui::SameLine(0.0F, RuntimeControlSpacing * 2.0F);
            ImGui::AlignTextToFramePadding();
            ImGui::TextDisabled("30 Hz fixed tick");
          })]
      .Render();

  ImGui::Spacing();
}
} // namespace gui
