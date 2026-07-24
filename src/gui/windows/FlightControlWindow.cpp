#include "gui/windows/FlightControlWindow.hpp"

#include "flightui/controls/Button.hpp"
#include "flightui/controls/Heading.hpp"
#include "flightui/controls/Slider.hpp"
#include "flightui/controls/Text.hpp"
#include "flightui/FlightUI.hpp"
#include "flightui/layout/HorizontalLayout.hpp"
#include "flightui/layout/VerticalLayout.hpp"
#include "control/ControlInput.hpp"
#include "gui/GUI.hpp"
#include "simulation/FlightDynamics.hpp"

#include <imgui.h>

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr double InputStep = 0.05;
constexpr float InputLayoutSpacing = 6.0F;
constexpr float InputRowSpacing = 8.0F;
constexpr float ButtonWidth = 32.0F;
constexpr float SliderWidth = 240.0F;

bool WasShortcutPressed(ImGuiKey key) { return ImGui::IsKeyPressed(key, true); }

bool CanApplyShortcuts() {
  return ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
         !ImGui::GetIO().WantTextInput;
}

void ApplyKeyboardShortcuts(sim::FlightDynamics &flightDynamics) {
  if (!CanApplyShortcuts()) {
    return;
  }

  const control::ControlInput &input = flightDynamics.GetControlInput();

  if (WasShortcutPressed(ImGuiKey_F)) {
    flightDynamics.SetThrottleInput(input.throttle - InputStep);
  }
  if (WasShortcutPressed(ImGuiKey_R)) {
    flightDynamics.SetThrottleInput(input.throttle + InputStep);
  }
  if (WasShortcutPressed(ImGuiKey_W)) {
    flightDynamics.SetElevatorInput(input.elevator - InputStep);
  }
  if (WasShortcutPressed(ImGuiKey_S)) {
    flightDynamics.SetElevatorInput(input.elevator + InputStep);
  }
  if (WasShortcutPressed(ImGuiKey_A)) {
    flightDynamics.SetAileronInput(input.aileron - InputStep);
  }
  if (WasShortcutPressed(ImGuiKey_D)) {
    flightDynamics.SetAileronInput(input.aileron + InputStep);
  }
  if (WasShortcutPressed(ImGuiKey_Q)) {
    flightDynamics.SetRudderInput(input.rudder - InputStep);
  }
  if (WasShortcutPressed(ImGuiKey_E)) {
    flightDynamics.SetRudderInput(input.rudder + InputStep);
  }
}

UI::UIElement MakeThrottleRow(sim::FlightDynamics &flightDynamics,
                              const control::ControlInput &input) {
  // clang-format off
  return UI::HorizontalLayout()
      .Spacing(InputRowSpacing)
      [
        +UI::Text("Throttle")
        + UI::Button("F")
              .OnAction([&flightDynamics] {
                flightDynamics.SetThrottleInput(
                    flightDynamics.GetControlInput().throttle - InputStep);
              })
              .Width(ButtonWidth)
        + UI::SliderDouble("##ThrottleInput", input.throttle, 0.0, 1.0)
              .OnChanged([&flightDynamics](double value) {
                flightDynamics.SetThrottleInput(value);
              })
              .Width(SliderWidth)
        + UI::Button("R")
              .OnAction([&flightDynamics] {
                flightDynamics.SetThrottleInput(
                    flightDynamics.GetControlInput().throttle + InputStep);
              })
              .Width(ButtonWidth)
      ];
  // clang-format on
}

UI::UIElement MakeElevatorRow(sim::FlightDynamics &flightDynamics,
                              const control::ControlInput &input) {
  // clang-format off
  return UI::HorizontalLayout()
      .Spacing(InputRowSpacing)
      [
        +UI::Text("Elevator")
        + UI::Button("W")
              .OnAction([&flightDynamics] {
                flightDynamics.SetElevatorInput(
                    flightDynamics.GetControlInput().elevator - InputStep);
              })
              .Width(ButtonWidth)
        + UI::SliderDouble("##ElevatorInput", input.elevator, -1.0, 1.0)
              .OnChanged([&flightDynamics](double value) {
                flightDynamics.SetElevatorInput(value);
              })
              .Width(SliderWidth)
        + UI::Button("S")
              .OnAction([&flightDynamics] {
                flightDynamics.SetElevatorInput(
                    flightDynamics.GetControlInput().elevator + InputStep);
              })
              .Width(ButtonWidth)
      ];
  // clang-format on
}

UI::UIElement MakeAileronRow(sim::FlightDynamics &flightDynamics,
                             const control::ControlInput &input) {
  // clang-format off
  return UI::HorizontalLayout()
      .Spacing(InputRowSpacing)
      [
        +UI::Text("Aileron")
        + UI::Button("A")
              .OnAction([&flightDynamics] {
                flightDynamics.SetAileronInput(
                    flightDynamics.GetControlInput().aileron - InputStep);
              })
              .Width(ButtonWidth)
        + UI::SliderDouble("##AileronInput", input.aileron, -1.0, 1.0)
              .OnChanged([&flightDynamics](double value) {
                flightDynamics.SetAileronInput(value);
              })
              .Width(SliderWidth)
        + UI::Button("D")
              .OnAction([&flightDynamics] {
                flightDynamics.SetAileronInput(
                    flightDynamics.GetControlInput().aileron + InputStep);
              })
              .Width(ButtonWidth)
      ];
  // clang-format on
}

UI::UIElement MakeRudderRow(sim::FlightDynamics &flightDynamics,
                            const control::ControlInput &input) {
  // clang-format off
  return UI::HorizontalLayout()
      .Spacing(InputRowSpacing)
      [
        +UI::Text("Rudder")
        + UI::Button("Q")
              .OnAction([&flightDynamics] {
                flightDynamics.SetRudderInput(
                    flightDynamics.GetControlInput().rudder - InputStep);
              })
              .Width(ButtonWidth)
        + UI::SliderDouble("##RudderInput", input.rudder, -1.0, 1.0)
              .OnChanged([&flightDynamics](double value) {
                flightDynamics.SetRudderInput(value);
              })
              .Width(SliderWidth)
        + UI::Button("E")
              .OnAction([&flightDynamics] {
                flightDynamics.SetRudderInput(
                    flightDynamics.GetControlInput().rudder + InputStep);
              })
              .Width(ButtonWidth)
      ];
  // clang-format on
}

UI::UIElement MakeControlInputLayout(sim::FlightDynamics &flightDynamics,
                                     const control::ControlInput &input) {
  // clang-format off
  return UI::VerticalLayout()
      .Spacing(InputLayoutSpacing)
      [
        +UI::Text("Control Inputs")
        + MakeThrottleRow(flightDynamics, input)
        + MakeElevatorRow(flightDynamics, input)
        + MakeAileronRow(flightDynamics, input)
        + MakeRudderRow(flightDynamics, input)
      ];
  // clang-format on
}
} // namespace

FlightControlWindow::FlightControlWindow() : Window("Flight Control") {}

void FlightControlWindow::OnUpdate(GUI &gui) {
  auto &simulation = gui.GetSimulation();
  auto &flightDynamics = simulation.GetFlightDynamics();
  ApplyKeyboardShortcuts(flightDynamics);

  const auto &controls = flightDynamics.GetFlightControls();
  const auto &input = flightDynamics.GetControlInput();
  const double throttle = controls.GetThrottle();
  const double elevator = controls.GetElevator();
  const double aileron = controls.GetAileron();
  const double rudder = controls.GetRudder();

  // clang-format off
  FlightUI::UIElement content =
      UI::VerticalLayout()
      [
        + MakeControlInputLayout(flightDynamics, input)
      ];

  // clang-format on

  content.Render();
}

} // namespace gui
