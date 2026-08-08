#include "application/gui/panels/AutopilotPanel.hpp"

#include "flightui/FlightUI.hpp"

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr float AutopilotTargetInputWidth = 140.0F;
constexpr float AutopilotParameterIndent = 24.0F;
constexpr float AutopilotParameterSliderWidth = 240.0F;
constexpr float HoldCaptureButtonWidth = 96.0F;
constexpr double MinimumPitchNaturalFrequencyRadPerSec = 4.0;

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
  bool preparing = false;
  const std::function<void()> &captureCurrent;
  const char *responseLabel = "";
  const char *responseId = "";
  const char *dampingRatioSliderId = "";
  double &dampingRatio;
  const char *naturalFrequencySliderId = "";
  double &naturalFrequencyRadPerSec;
  double minimumNaturalFrequencyRadPerSec = 0.1;
  bool &responseOpen;
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

UI::UIElement MakeAutopilotParameterSlider(const char *label,
    const char *sliderId, double &value, double minimum, double maximum) {
  return UI::HorizontalLayout().Spacing(
      8.0F)[+UI::HorizontalSpace(AutopilotParameterIndent)
            + UI::TextDisabled(label)
            + UI::SliderDouble(sliderId, value, minimum, maximum)
                .Width(AutopilotParameterSliderWidth)
                .Format("%.2f")
                .OnChanged([&value](double newValue) { value = newValue; })];
}

UI::UIElement MakeAxisHoldStatusRow(const AxisHoldSectionConfig &config) {
  // clang-format off
  return UI::HorizontalLayout()
      .Spacing(8.0F)
      [
        +UI::ValueLabel(config.currentLabel, config.currentValue, "%.2f deg")
        + UI::ValueLabel(config.rateLabel, config.rateValue, "%.2f deg/s")
        + UI::ValueLabel(config.outputLabel, config.outputValue, "%.3f")
        + UI::Text(config.active
                       ? "Active"
                       : (config.preparing ? "Preparing" : "Inactive"))
        + UI::Button("Capture")
              .Enabled(static_cast<bool>(config.captureCurrent))
              .OnAction(config.captureCurrent)
              .Width(HoldCaptureButtonWidth)
      ];
  // clang-format on
}

UI::UIElement MakeAxisHoldResponseFoldOut(const AxisHoldSectionConfig &config) {
  return UI::FoldOut(config.responseLabel)
      .Open(config.responseOpen)
      .SpanAvailWidth()
      .Id(config.responseId)[UI::VerticalLayout().Spacing(
          6.0F)[+MakeAutopilotParameterSlider("Damping Ratio",
                    config.dampingRatioSliderId,
                    config.dampingRatio,
                    0.1,
                    2.0)
                + MakeAutopilotParameterSlider("Natural Frequency (rad/s)",
                    config.naturalFrequencySliderId,
                    config.naturalFrequencyRadPerSec,
                    config.minimumNaturalFrequencyRadPerSec,
                    10.0)]];
}

UI::UIElement MakeAxisHoldSection(const AxisHoldSectionConfig &config) {
  UI::VerticalLayoutBuilder layout = UI::VerticalLayout().Spacing(6.0F)
                                     + MakeAutopilotHoldRow(config.holdLabel,
                                         config.targetLabel,
                                         config.targetInputId,
                                         config.enabled,
                                         config.targetValue)
                                     + MakeAxisHoldStatusRow(config);

  if (config.enabled) {
    layout = layout + MakeAxisHoldResponseFoldOut(config);
  }

  return layout;
}

UI::UIElement MakeRollHoldSection(const AutopilotPanelProps &props) {
  AutopilotPanelState &state = props.state;
  return MakeAxisHoldSection({
      .holdLabel = "Roll Hold",
      .targetLabel = "Target Roll (deg)",
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
      .preparing = props.rollHoldPreparing,
      .captureCurrent = props.captureCurrentRoll,
      .responseLabel = "Roll Hold Response",
      .responseId = "RollHoldResponse",
      .dampingRatioSliderId = "##RollHoldDampingRatio",
      .dampingRatio = state.rollHoldDampingRatio,
      .naturalFrequencySliderId = "##RollHoldNaturalFrequency",
      .naturalFrequencyRadPerSec = state.rollHoldNaturalFrequencyRadPerSec,
      .responseOpen = state.rollHoldResponseOpen,
  });
}

UI::UIElement MakePitchHoldSection(const AutopilotPanelProps &props) {
  AutopilotPanelState &state = props.state;
  return MakeAxisHoldSection({
      .holdLabel = "Pitch Hold",
      .targetLabel = "Target Pitch (deg)",
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
      .preparing = props.pitchHoldPreparing,
      .captureCurrent = props.captureCurrentPitch,
      .responseLabel = "Pitch Hold Response",
      .responseId = "PitchHoldResponse",
      .dampingRatioSliderId = "##PitchHoldDampingRatio",
      .dampingRatio = state.pitchHoldDampingRatio,
      .naturalFrequencySliderId = "##PitchHoldNaturalFrequency",
      .naturalFrequencyRadPerSec = state.pitchHoldNaturalFrequencyRadPerSec,
      .minimumNaturalFrequencyRadPerSec = MinimumPitchNaturalFrequencyRadPerSec,
      .responseOpen = state.pitchHoldResponseOpen,
  });
}
} // namespace

void AutopilotPanel::Draw(AutopilotPanelState &state) {
  Draw({.state = state});
}

void AutopilotPanel::Draw(const AutopilotPanelProps &props) {
  AutopilotPanelState &state = props.state;
  UI::VerticalLayout()
      .Spacing(8.0F)[+UI::Heading("Autopilot") + MakeRollHoldSection(props)
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
