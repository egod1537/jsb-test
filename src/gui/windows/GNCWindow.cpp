#include "GNCWindow.hpp"
#include "control/ControlInput.hpp"
#include "control/ControlInputStrategy.hpp"
#include "flightui/FlightUI.hpp"
#include "gui/GUI.hpp"
#include "simulation/Aircraft.hpp"
#include "simulation/Simulation.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr float TrimInputPanelHeight = 178.0F;
constexpr float TrimLayoutSpacing = 8.0F;
constexpr float TrimButtonWidth = 160.0F;
constexpr double ManualInputStep = 0.05;
constexpr float ManualInputLayoutSpacing = 6.0F;
constexpr float ManualInputRowSpacing = 8.0F;
constexpr float ManualInputButtonWidth = 32.0F;
constexpr float ManualInputSliderWidth = 240.0F;
constexpr float AutopilotTargetInputWidth = 140.0F;
constexpr float AutopilotGainIndent = 24.0F;
constexpr float AutopilotGainSliderWidth = 240.0F;

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
    const control::ControlInput &input) {
  // clang-format off
  return UI::VerticalLayout()
      .Spacing(ManualInputLayoutSpacing)
      [
        +UI::Text("Control Inputs")
        + MakeThrottleRow(strategy, autopilotState, input)
        + MakeElevatorRow(strategy, autopilotState, input)
        + MakeAileronRow(strategy, autopilotState, input)
        + MakeRudderRow(strategy, autopilotState, input)
      ];
  // clang-format on
}

void DrawFlightControlTab(sim::Aircraft &aircraft,
    const AutopilotPanelState &autopilotState) {
  control::ControlInputStrategy &strategy = aircraft.GetControlInputStrategy();
  ApplyManualInputShortcuts(strategy, autopilotState);
  MakeManualInputLayout(strategy, autopilotState, strategy.GetCommandedInput())
      .Render();
}

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

UI::UIElement MakeAutopilotHoldRow(const char *holdLabel,
    const char *targetLabel, const char *inputId, bool &enabled,
    double &targetValue, double step = 1.0, double fastStep = 10.0) {
  return UI::HorizontalLayout().Spacing(
      8.0F)[+UI::Toggle(holdLabel, enabled).OnChanged([&enabled](bool value) {
    enabled = value;
  }) + UI::TextDisabled(targetLabel)
            + UI::InputDouble(inputId, targetValue)
                .Width(AutopilotTargetInputWidth)
                .Step(step)
                .FastStep(fastStep)
                .Format("%.2f")
                .OnChanged(
                    [&targetValue](double value) { targetValue = value; })
            + UI::Text(enabled ? "Hold" : "Off")];
}

UI::UIElement MakeAutopilotGainSlider(const char *label, const char *sliderId,
    double &value, double minimum, double maximum) {
  return UI::HorizontalLayout().Spacing(
      8.0F)[+UI::HorizontalSpace(AutopilotGainIndent)
            + UI::TextDisabled(label)
            + UI::SliderDouble(sliderId, value, minimum, maximum)
                  .Width(AutopilotGainSliderWidth)
                  .Format("%.2f")
                  .OnChanged([&value](double newValue) {
                    value = newValue;
                  })];
}

UI::UIElement MakeRollHoldSection(AutopilotPanelState &state) {
  UI::VerticalLayoutBuilder layout =
      UI::VerticalLayout().Spacing(6.0F)
      + MakeAutopilotHoldRow("Roll Hold",
          "Roll (deg)",
          "##RollHoldTarget",
          state.rollHold,
          state.rollTargetDeg);

  if (state.rollHold) {
    layout = layout
             + MakeAutopilotGainSlider(
                 "k_p", "##RollHoldKp", state.rollHoldKp, 0.1, 5.0)
             + MakeAutopilotGainSlider(
                 "k_d", "##RollHoldKd", state.rollHoldKd, 0.02, 2.0);
  }

  return layout;
}

void DrawAutopilotTab(AutopilotPanelState &state) {
  UI::VerticalLayout()
      .Spacing(8.0F)[+UI::Heading("Autopilot")
                     + MakeRollHoldSection(state)
                     + MakeAutopilotHoldRow("Pitch Hold",
                         "Pitch (deg)",
                         "##PitchHoldTarget",
                         state.pitchHold,
                         state.pitchTargetDeg)
                     + MakeAutopilotHoldRow("Yaw Hold",
                         "Yaw (deg)",
                         "##YawHoldTarget",
                         state.yawHold,
                         state.yawTargetDeg)
                     + MakeAutopilotHoldRow("Altitude Hold",
                         "Altitude (ft)",
                         "##AltitudeHoldTarget",
                         state.altitudeHold,
                         state.altitudeTargetFt,
                         100.0,
                         1000.0)
                     + MakeAutopilotHoldRow("Course Hold",
                         "Course (deg)",
                         "##CourseHoldTarget",
                         state.courseHold,
                         state.courseTargetDeg)]
      .Render();
}

UI::UIElement MakeTrimInputPanel(gnc::TrimRequest &request) {
  return UI::Panel("TrimInputPanel")
      .FlexibleWidth(true)
      .Height(TrimInputPanelHeight)
      .Border(true)[UI::VerticalLayout().Spacing(
          6.0F)[+UI::Heading("Trim Input")
                + UI::Combo("Mode",
                    TrimModeIndex(request.mode),
                    {"Longitudinal", "Full", "Ground"})
                    .OnChanged([&request](int index) {
                      request.mode = TrimModeFromIndex(index);
                    })
                + UI::InputDouble("Airspeed (kt)", request.airspeedKts)
                    .Step(1.0)
                    .FastStep(10.0)
                    .Format("%.2f")
                    .OnChanged([&request](double value) {
                      request.airspeedKts = value;
                    })
                + UI::InputDouble("Altitude (ft)", request.altitudeFt)
                    .Step(100.0)
                    .FastStep(1000.0)
                    .Format("%.2f")
                    .OnChanged([&request](double value) {
                      request.altitudeFt = value;
                    })
                + UI::InputDouble("Flight Path Angle (deg)",
                    request.flightPathAngleDeg)
                    .Step(0.1)
                    .FastStep(1.0)
                    .Format("%.2f")
                    .OnChanged([&request](double value) {
                      request.flightPathAngleDeg = value;
                    })]];
}

UI::UIElement MakeTrimRequestSummary(const gnc::TrimRequest &request) {
  return UI::KeyValueGrid("TrimRequestSummaryTable")
      .ColumnsPerRow(4)
      .Add("Mode", TrimModeLabel(request.mode))
      .AddDouble("Airspeed", request.airspeedKts, "%.2f kt")
      .AddDouble("Altitude", request.altitudeFt, "%.2f ft")
      .AddDouble("Flight Path Angle", request.flightPathAngleDeg, "%.2f deg");
}

UI::UIElement MakeTrimResultContent(const gnc::TrimResult &result,
    bool hasResult) {
  UI::VerticalLayoutBuilder layout =
      UI::VerticalLayout().Spacing(6.0F)
      + UI::HorizontalLayout().Spacing(
          6.0F)[+UI::TextDisabled("Status")
                + UI::Text(hasResult ? (result.success ? "Success" : "Failed")
                                     : "Idle")];

  if (!result.message.empty()) {
    layout = layout
             + UI::HorizontalLayout().Spacing(
                 6.0F)[+UI::TextDisabled("Message")
                       + UI::TextWrapped(result.message)];
  }

  layout = layout
           + UI::KeyValueGrid("TrimResultMetrics")
                 .ColumnsPerRow(2)
                 .AddDouble("Alpha", result.alphaDeg, "%.2f deg")
                 .AddDouble("Beta", result.betaDeg, "%.2f deg")
                 .AddDouble("Roll", result.rollDeg, "%.2f deg")
                 .AddDouble("Pitch", result.pitchDeg, "%.2f deg")
                 .AddDouble("Throttle", result.throttle, "%.3f")
                 .AddDouble("Elevator", result.elevator, "%.3f")
                 .AddDouble("Pitch Trim", result.pitchTrim, "%.3f")
                 .AddDouble("Aileron", result.aileron, "%.3f")
                 .AddDouble("Rudder", result.rudder, "%.3f");

  return layout;
}

UI::UIElement MakeTrimResidualContent(const gnc::TrimResult &result) {
  return UI::KeyValueGrid("TrimResidualMetrics")
      .ColumnsPerRow(2)
      .AddDouble("uDot", result.uDot, "%.4f m/s^2")
      .AddDouble("vDot", result.vDot, "%.4f m/s^2")
      .AddDouble("wDot", result.wDot, "%.4f m/s^2")
      .AddDouble("pDot", result.pDot, "%.4f deg/s^2")
      .AddDouble("qDot", result.qDot, "%.4f deg/s^2")
      .AddDouble("rDot", result.rDot, "%.4f deg/s^2");
}

void DrawTrimTab(gnc::TrimRequest &request, gnc::TrimResult &result,
    bool &hasResult, bool &resultOpen, bool &residualOpen, bool canRequestTrim,
    const UI::Action &requestRunICTrim,
    const UI::Action &requestCurrentStateTrim) {
  UI::VerticalLayout()
      .Spacing(TrimLayoutSpacing)
          [+MakeTrimInputPanel(request)
              + UI::HorizontalLayout().Spacing(
                  8.0F)[+UI::Button("RunIC Trim")
                            .Width(TrimButtonWidth)
                            .Enabled(canRequestTrim)
                            .OnAction(requestRunICTrim)
                        + UI::Button("Current State Trim")
                            .Width(TrimButtonWidth)
                            .Enabled(canRequestTrim)
                            .OnAction(requestCurrentStateTrim)]
              + MakeTrimRequestSummary(request) + UI::Space(6.0F)
              + UI::FoldOut("Result").Open(
                  resultOpen)[MakeTrimResultContent(result, hasResult)]
              + UI::FoldOut("Residual")
                  .Open(residualOpen)[MakeTrimResidualContent(result)]]
      .Render();
}
} // namespace

GNCWindow::GNCWindow() : Window("GNC") {}

void GNCWindow::RequestTrim(PendingTrimCommand command) {
  if (pendingTrimCommand_ != PendingTrimCommand::None || trimInProgress_) {
    return;
  }

  pendingTrimCommand_ = command;
}

void GNCWindow::ExecutePendingTrim(sim::Simulation &simulation) {
  const PendingTrimCommand command = pendingTrimCommand_;
  if (command == PendingTrimCommand::None || trimInProgress_) {
    return;
  }

  pendingTrimCommand_ = PendingTrimCommand::None;
  trimInProgress_ = true;

  auto &aircraft = simulation.GetAircraft();
  const double simTime = aircraft.GetAircraftState().simulationTimeSec;
  const bool resumeAfterTrim = simulation.IsRunning();
  simulation.Pause();

  const char *commandLabel = "None";
  switch (command) {
  case PendingTrimCommand::RunInitialCondition:
    commandLabel = "RunIC";
    break;
  case PendingTrimCommand::CurrentState:
    commandLabel = "CurrentState";
    break;
  case PendingTrimCommand::None:
  default:
    break;
  }

  std::cout << "[GNC] trim request command=" << commandLabel
            << " simTime=" << simTime << '\n';

  if (command == PendingTrimCommand::RunInitialCondition) {
    trimResult_ = trimSolver_.Trim(aircraft, trimRequest_);
  } else if (command == PendingTrimCommand::CurrentState) {
    trimResult_ = trimSolver_.TrimCurrentState(aircraft, trimRequest_.mode);
  }

  trimHasResult_ = true;
  trimResultOpen_ = true;
  trimResidualOpen_ = true;
  trimInProgress_ = false;

  if (resumeAfterTrim) {
    simulation.Resume();
  }
}

void GNCWindow::OnUpdate(gui::GUI &gui) {
  auto &simulation = gui.GetSimulation();
  auto &aircraft = simulation.GetAircraft();

  UI::VerticalLayout()[+UI::TabGroup("GNC")[+UI::Tab("Trim")[UI::Custom([this] {
    DrawTrimTab(
        trimRequest_,
        trimResult_,
        trimHasResult_,
        trimResultOpen_,
        trimResidualOpen_,
        !trimInProgress_ && pendingTrimCommand_ == PendingTrimCommand::None,
        [this] { RequestTrim(PendingTrimCommand::RunInitialCondition); },
        [this] { RequestTrim(PendingTrimCommand::CurrentState); });
  })] + UI::Tab("Autopilot")[UI::Custom([this] {
    DrawAutopilotTab(autopilotPanelState_);
  })] + UI::Tab("Flight Control")[UI::Custom([&aircraft, this] {
    DrawFlightControlTab(aircraft, autopilotPanelState_);
  })]]]
      .Render();

  ExecutePendingTrim(simulation);
}
} // namespace gui
