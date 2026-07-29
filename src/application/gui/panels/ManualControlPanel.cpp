#include "application/gui/panels/ManualControlPanel.hpp"

#include "application/sim/Aircraft.hpp"
#include "application/sim/control/ControlInput.hpp"
#include "application/sim/control/ControlInputStrategy.hpp"
#include "flightui/FlightUI.hpp"

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr double ManualInputStep = 0.05;
constexpr float ManualInputLayoutSpacing = 6.0F;
constexpr float ManualInputRowSpacing = 8.0F;
constexpr float ManualInputButtonWidth = 32.0F;
constexpr float ManualInputSliderWidth = 240.0F;

bool WasShortcutPressed(UI::Key key) { return UI::IsKeyPressed(key, true); }

bool CanApplyManualInputShortcuts() {
  return UI::IsCurrentWindowFocused() && !UI::WantsTextInput();
}

bool IsManualControlAllowed(const AutopilotPanelState &autopilotState,
    control::ControlAxis axis) {
  switch (axis) {
  case control::ControlAxis::Elevator:
    return !autopilotState.pitchHold && !autopilotState.altitudeHold;
  case control::ControlAxis::Aileron:
    return !autopilotState.rollHold && !autopilotState.courseHold;
  case control::ControlAxis::Rudder:
    return !autopilotState.yawHold;
  case control::ControlAxis::Throttle:
    return true;
  }

  return true;
}

const char *ManualControlLockTooltip(const AutopilotPanelState &autopilotState,
    control::ControlAxis axis) {
  switch (axis) {
  case control::ControlAxis::Elevator:
    if (autopilotState.altitudeHold) {
      return "Altitude Hold is controlling elevator.";
    }
    if (autopilotState.pitchHold) {
      return "Pitch Hold is controlling elevator.";
    }
    break;
  case control::ControlAxis::Aileron:
    if (autopilotState.courseHold) {
      return "Course Hold is controlling aileron.";
    }
    if (autopilotState.rollHold) {
      return "Roll Hold is controlling aileron.";
    }
    break;
  case control::ControlAxis::Rudder:
    if (autopilotState.yawHold) {
      return "Yaw Hold is controlling rudder.";
    }
    break;
  case control::ControlAxis::Throttle:
    break;
  }

  return "";
}

void AdjustManualInput(control::ControlInputStrategy &strategy,
    const AutopilotPanelState &autopilotState, control::ControlAxis axis,
    double delta) {
  if (!IsManualControlAllowed(autopilotState, axis)) {
    return;
  }

  strategy.AdjustCommandedInput(axis, delta);
}

void SetManualInput(control::ControlInputStrategy &strategy,
    const AutopilotPanelState &autopilotState, control::ControlAxis axis,
    double value) {
  if (!IsManualControlAllowed(autopilotState, axis)) {
    return;
  }

  strategy.SetCommandedInput(axis, value);
}

void ApplyManualInputShortcuts(control::ControlInputStrategy &strategy,
    const AutopilotPanelState &autopilotState) {
  if (!CanApplyManualInputShortcuts()) {
    return;
  }

  if (WasShortcutPressed(UI::Key::F)) {
    AdjustManualInput(strategy,
        autopilotState,
        control::ControlAxis::Throttle,
        -ManualInputStep);
  }
  if (WasShortcutPressed(UI::Key::R)) {
    AdjustManualInput(strategy,
        autopilotState,
        control::ControlAxis::Throttle,
        ManualInputStep);
  }
  if (WasShortcutPressed(UI::Key::W)) {
    AdjustManualInput(strategy,
        autopilotState,
        control::ControlAxis::Elevator,
        -ManualInputStep);
  }
  if (WasShortcutPressed(UI::Key::S)) {
    AdjustManualInput(strategy,
        autopilotState,
        control::ControlAxis::Elevator,
        ManualInputStep);
  }
  if (WasShortcutPressed(UI::Key::A)) {
    AdjustManualInput(strategy,
        autopilotState,
        control::ControlAxis::Aileron,
        -ManualInputStep);
  }
  if (WasShortcutPressed(UI::Key::D)) {
    AdjustManualInput(strategy,
        autopilotState,
        control::ControlAxis::Aileron,
        ManualInputStep);
  }
  if (WasShortcutPressed(UI::Key::Q)) {
    AdjustManualInput(strategy,
        autopilotState,
        control::ControlAxis::Rudder,
        -ManualInputStep);
  }
  if (WasShortcutPressed(UI::Key::E)) {
    AdjustManualInput(strategy,
        autopilotState,
        control::ControlAxis::Rudder,
        ManualInputStep);
  }
}

UI::UIElement MakeThrottleRow(control::ControlInputStrategy &strategy,
    const AutopilotPanelState &autopilotState,
    const control::ControlInput &input) {
  const bool enabled =
      IsManualControlAllowed(autopilotState, control::ControlAxis::Throttle);
  const char *tooltip =
      ManualControlLockTooltip(autopilotState, control::ControlAxis::Throttle);

  // clang-format off
  return UI::HorizontalLayout()
      .Spacing(ManualInputRowSpacing)
      [
        +UI::Text("Throttle")
        + UI::Button("F")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&strategy, &autopilotState] {
                AdjustManualInput(strategy,
                    autopilotState,
                    control::ControlAxis::Throttle,
                    -ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
        + UI::SliderDouble("##ThrottleInput", input.throttle, 0.0, 1.0)
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnChanged([&strategy, &autopilotState](double value) {
                SetManualInput(strategy,
                    autopilotState,
                    control::ControlAxis::Throttle,
                    value);
              })
              .Width(ManualInputSliderWidth)
        + UI::Button("R")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&strategy, &autopilotState] {
                AdjustManualInput(strategy,
                    autopilotState,
                    control::ControlAxis::Throttle,
                    ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
      ];
  // clang-format on
}

UI::UIElement MakeElevatorRow(control::ControlInputStrategy &strategy,
    const AutopilotPanelState &autopilotState,
    const control::ControlInput &input) {
  const bool enabled =
      IsManualControlAllowed(autopilotState, control::ControlAxis::Elevator);
  const char *tooltip =
      ManualControlLockTooltip(autopilotState, control::ControlAxis::Elevator);

  // clang-format off
  return UI::HorizontalLayout()
      .Spacing(ManualInputRowSpacing)
      [
        +UI::Text("Elevator")
        + UI::Button("W")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&strategy, &autopilotState] {
                AdjustManualInput(strategy,
                    autopilotState,
                    control::ControlAxis::Elevator,
                    -ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
        + UI::SliderDouble("##ElevatorInput", input.elevator, -1.0, 1.0)
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnChanged([&strategy, &autopilotState](double value) {
                SetManualInput(strategy,
                    autopilotState,
                    control::ControlAxis::Elevator,
                    value);
              })
              .Width(ManualInputSliderWidth)
        + UI::Button("S")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&strategy, &autopilotState] {
                AdjustManualInput(strategy,
                    autopilotState,
                    control::ControlAxis::Elevator,
                    ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
      ];
  // clang-format on
}

UI::UIElement MakeAileronRow(control::ControlInputStrategy &strategy,
    const AutopilotPanelState &autopilotState,
    const control::ControlInput &input) {
  const bool enabled =
      IsManualControlAllowed(autopilotState, control::ControlAxis::Aileron);
  const char *tooltip =
      ManualControlLockTooltip(autopilotState, control::ControlAxis::Aileron);

  // clang-format off
  return UI::HorizontalLayout()
      .Spacing(ManualInputRowSpacing)
      [
        +UI::Text("Aileron")
        + UI::Button("A")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&strategy, &autopilotState] {
                AdjustManualInput(strategy,
                    autopilotState,
                    control::ControlAxis::Aileron,
                    -ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
        + UI::SliderDouble("##AileronInput", input.aileron, -1.0, 1.0)
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnChanged([&strategy, &autopilotState](double value) {
                SetManualInput(strategy,
                    autopilotState,
                    control::ControlAxis::Aileron,
                    value);
              })
              .Width(ManualInputSliderWidth)
        + UI::Button("D")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&strategy, &autopilotState] {
                AdjustManualInput(strategy,
                    autopilotState,
                    control::ControlAxis::Aileron,
                    ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
      ];
  // clang-format on
}

UI::UIElement MakeRudderRow(control::ControlInputStrategy &strategy,
    const AutopilotPanelState &autopilotState,
    const control::ControlInput &input) {
  const bool enabled =
      IsManualControlAllowed(autopilotState, control::ControlAxis::Rudder);
  const char *tooltip =
      ManualControlLockTooltip(autopilotState, control::ControlAxis::Rudder);

  // clang-format off
  return UI::HorizontalLayout()
      .Spacing(ManualInputRowSpacing)
      [
        +UI::Text("Rudder")
        + UI::Button("Q")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&strategy, &autopilotState] {
                AdjustManualInput(strategy,
                    autopilotState,
                    control::ControlAxis::Rudder,
                    -ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
        + UI::SliderDouble("##RudderInput", input.rudder, -1.0, 1.0)
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnChanged([&strategy, &autopilotState](double value) {
                SetManualInput(strategy,
                    autopilotState,
                    control::ControlAxis::Rudder,
                    value);
              })
              .Width(ManualInputSliderWidth)
        + UI::Button("E")
              .Enabled(enabled)
              .Tooltip(tooltip)
              .OnAction([&strategy, &autopilotState] {
                AdjustManualInput(strategy,
                    autopilotState,
                    control::ControlAxis::Rudder,
                    ManualInputStep);
              })
              .Width(ManualInputButtonWidth)
      ];
  // clang-format on
}

UI::UIElement MakeManualInputLayout(control::ControlInputStrategy &strategy,
    const AutopilotPanelState &autopilotState,
    const control::ControlInput &input, double pitchTrim) {
  // clang-format off
  return UI::VerticalLayout()
      .Spacing(ManualInputLayoutSpacing)
      [
        +UI::Text("Control Inputs")
        + MakeThrottleRow(strategy, autopilotState, input)
        + MakeElevatorRow(strategy, autopilotState, input)
        + MakeAileronRow(strategy, autopilotState, input)
        + MakeRudderRow(strategy, autopilotState, input)
        + UI::ValueLabel("Pitch Trim", pitchTrim, "%.3f")
      ];
  // clang-format on
}
} // namespace

void ManualControlPanel::Draw(sim::Aircraft &aircraft,
    const AutopilotPanelState &autopilotState) {
  control::ControlInputStrategy &strategy = aircraft.GetControlInputStrategy();
  ApplyManualInputShortcuts(strategy, autopilotState);
  MakeManualInputLayout(strategy,
      autopilotState,
      strategy.GetCommandedInput(),
      aircraft.GetFlightControls().GetPitchTrim())
      .Render();
}
} // namespace gui
