#include "application/gui/panels/AutopilotPanel.hpp"

#include "flightui/FlightUI.hpp"

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr float AutopilotTargetInputWidth = 140.0F;
constexpr float AutopilotGainIndent = 24.0F;
constexpr float AutopilotGainSliderWidth = 240.0F;
constexpr float HoldCaptureButtonWidth = 96.0F;

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

UI::UIElement MakeRollHoldStatusRow(const AutopilotPanelProps &props) {
  // clang-format off
  return UI::HorizontalLayout()
      .Spacing(8.0F)
      [
        +UI::ValueLabel("Current Roll", props.currentRollDeg, "%.2f deg")
        + UI::ValueLabel("Roll Rate",
              props.currentRollRateDegPerSec,
              "%.2f deg/s")
        + UI::ValueLabel("Aileron", props.currentAileron, "%.3f")
        + UI::Text(props.rollHoldActive ? "Active" : "Inactive")
        + UI::Button("Capture")
              .Enabled(static_cast<bool>(props.captureCurrentRoll))
              .OnAction(props.captureCurrentRoll)
              .Width(HoldCaptureButtonWidth)
      ];
  // clang-format on
}

UI::UIElement MakeRollHoldSection(const AutopilotPanelProps &props) {
  AutopilotPanelState &state = props.state;
  UI::VerticalLayoutBuilder layout =
      UI::VerticalLayout().Spacing(6.0F)
      + MakeAutopilotHoldRow("Roll Hold",
          "Roll (deg)",
          "##RollHoldTarget",
          state.rollHold,
          state.rollTargetDeg)
      + MakeRollHoldStatusRow(props);

  if (state.rollHold) {
    layout =
        layout
        + UI::FoldOut("Roll Hold Gains")
              .Open(state.rollHoldGainsOpen)
              .SpanAvailWidth()
              .Id("RollHoldGains")
              [UI::VerticalLayout().Spacing(6.0F)
                  [+MakeAutopilotGainSlider(
                       "k_p", "##RollHoldKp", state.rollHoldKp, 0.1, 5.0)
                      + MakeAutopilotGainSlider(
                          "k_d", "##RollHoldKd", state.rollHoldKd, 0.02, 2.0)]];
  }

  return layout;
}

UI::UIElement MakePitchHoldStatusRow(const AutopilotPanelProps &props) {
  // clang-format off
  return UI::HorizontalLayout()
      .Spacing(8.0F)
      [
        +UI::ValueLabel("Current Pitch", props.currentPitchDeg, "%.2f deg")
        + UI::ValueLabel("Pitch Rate",
              props.currentPitchRateDegPerSec,
              "%.2f deg/s")
        + UI::ValueLabel("Elevator", props.currentElevator, "%.3f")
        + UI::Text(props.pitchHoldActive ? "Active" : "Inactive")
        + UI::Button("Capture")
              .Enabled(static_cast<bool>(props.captureCurrentPitch))
              .OnAction(props.captureCurrentPitch)
              .Width(HoldCaptureButtonWidth)
      ];
  // clang-format on
}

UI::UIElement MakePitchHoldSection(const AutopilotPanelProps &props) {
  AutopilotPanelState &state = props.state;
  UI::VerticalLayoutBuilder layout =
      UI::VerticalLayout().Spacing(6.0F)
      + MakeAutopilotHoldRow("Pitch Hold",
          "Pitch (deg)",
          "##PitchHoldTarget",
          state.pitchHold,
          state.pitchTargetDeg)
      + MakePitchHoldStatusRow(props);

  if (state.pitchHold) {
    layout =
        layout
        + UI::FoldOut("Pitch Hold Gains")
              .Open(state.pitchHoldGainsOpen)
              .SpanAvailWidth()
              .Id("PitchHoldGains")
              [UI::VerticalLayout().Spacing(6.0F)
                  [+MakeAutopilotGainSlider(
                       "k_p", "##PitchHoldKp", state.pitchHoldKp, 0.1, 5.0)
                      + MakeAutopilotGainSlider("k_d",
                          "##PitchHoldKd",
                          state.pitchHoldKd,
                          0.02,
                          2.0)]];
  }

  return layout;
}
} // namespace

void AutopilotPanel::Draw(AutopilotPanelState &state) {
  Draw({.state = state});
}

void AutopilotPanel::Draw(const AutopilotPanelProps &props) {
  AutopilotPanelState &state = props.state;
  UI::VerticalLayout()
      .Spacing(8.0F)[+UI::Heading("Autopilot")
                     + MakeRollHoldSection(props)
                     + MakePitchHoldSection(props)
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
} // namespace gui
