#include "application/gui/panels/AutopilotPanel.hpp"

#include "flightui/FlightUI.hpp"

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr float AutopilotTargetInputWidth = 140.0F;
constexpr float AutopilotGainIndent = 24.0F;
constexpr float AutopilotGainSliderWidth = 240.0F;
constexpr float HoldCaptureButtonWidth = 96.0F;

struct AxisHoldSectionConfig {
  const char *holdLabel = "";
  const char *targetLabel = "";
  const char *targetInputId = "";
  bool &enabled;
  double &targetValue;
  const char *currentLabel = "";
  double currentValue = 0.0;
  const char *rateLabel = "";
  double rateValue = 0.0;
  const char *outputLabel = "";
  double outputValue = 0.0;
  bool active = false;
  const std::function<void()> &captureCurrent;
  const char *gainsLabel = "";
  const char *gainsId = "";
  const char *kpSliderId = "";
  double &kp;
  const char *kdSliderId = "";
  double &kd;
  bool &gainsOpen;
};

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

UI::UIElement MakeAxisHoldStatusRow(
    const AxisHoldSectionConfig &config) {
  // clang-format off
  return UI::HorizontalLayout()
      .Spacing(8.0F)
      [
        +UI::ValueLabel(config.currentLabel, config.currentValue, "%.2f deg")
        + UI::ValueLabel(config.rateLabel, config.rateValue, "%.2f deg/s")
        + UI::ValueLabel(config.outputLabel, config.outputValue, "%.3f")
        + UI::Text(config.active ? "Active" : "Inactive")
        + UI::Button("Capture")
              .Enabled(static_cast<bool>(config.captureCurrent))
              .OnAction(config.captureCurrent)
              .Width(HoldCaptureButtonWidth)
      ];
  // clang-format on
}

UI::UIElement MakeAxisHoldGainsFoldOut(
    const AxisHoldSectionConfig &config) {
  return UI::FoldOut(config.gainsLabel)
      .Open(config.gainsOpen)
      .SpanAvailWidth()
      .Id(config.gainsId)
      [UI::VerticalLayout().Spacing(6.0F)
          [+MakeAutopilotGainSlider(
               "k_p", config.kpSliderId, config.kp, 0.1, 5.0)
              + MakeAutopilotGainSlider(
                  "k_d", config.kdSliderId, config.kd, 0.02, 2.0)]];
}

UI::UIElement MakeAxisHoldSection(const AxisHoldSectionConfig &config) {
  UI::VerticalLayoutBuilder layout =
      UI::VerticalLayout().Spacing(6.0F)
      + MakeAutopilotHoldRow(config.holdLabel,
          config.targetLabel,
          config.targetInputId,
          config.enabled,
          config.targetValue)
      + MakeAxisHoldStatusRow(config);

  if (config.enabled) {
    layout = layout + MakeAxisHoldGainsFoldOut(config);
  }

  return layout;
}

UI::UIElement MakeRollHoldSection(const AutopilotPanelProps &props) {
  AutopilotPanelState &state = props.state;
  return MakeAxisHoldSection({
      .holdLabel = "Roll Hold",
      .targetLabel = "Roll (deg)",
      .targetInputId = "##RollHoldTarget",
      .enabled = state.rollHold,
      .targetValue = state.rollTargetDeg,
      .currentLabel = "Current Roll",
      .currentValue = props.currentRollDeg,
      .rateLabel = "Roll Rate",
      .rateValue = props.currentRollRateDegPerSec,
      .outputLabel = "Aileron",
      .outputValue = props.currentAileron,
      .active = props.rollHoldActive,
      .captureCurrent = props.captureCurrentRoll,
      .gainsLabel = "Roll Hold Gains",
      .gainsId = "RollHoldGains",
      .kpSliderId = "##RollHoldKp",
      .kp = state.rollHoldKp,
      .kdSliderId = "##RollHoldKd",
      .kd = state.rollHoldKd,
      .gainsOpen = state.rollHoldGainsOpen,
  });
}

UI::UIElement MakePitchHoldSection(const AutopilotPanelProps &props) {
  AutopilotPanelState &state = props.state;
  return MakeAxisHoldSection({
      .holdLabel = "Pitch Hold",
      .targetLabel = "Pitch (deg)",
      .targetInputId = "##PitchHoldTarget",
      .enabled = state.pitchHold,
      .targetValue = state.pitchTargetDeg,
      .currentLabel = "Current Pitch",
      .currentValue = props.currentPitchDeg,
      .rateLabel = "Pitch Rate",
      .rateValue = props.currentPitchRateDegPerSec,
      .outputLabel = "Elevator",
      .outputValue = props.currentElevator,
      .active = props.pitchHoldActive,
      .captureCurrent = props.captureCurrentPitch,
      .gainsLabel = "Pitch Hold Gains",
      .gainsId = "PitchHoldGains",
      .kpSliderId = "##PitchHoldKp",
      .kp = state.pitchHoldKp,
      .kdSliderId = "##PitchHoldKd",
      .kd = state.pitchHoldKd,
      .gainsOpen = state.pitchHoldGainsOpen,
  });
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
