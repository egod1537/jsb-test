#include "GNCWindow.hpp"

#include "application/gui/GUI.hpp"
#include "application/gui/panels/AutopilotPanel.hpp"
#include "application/gui/panels/ManualControlPanel.hpp"
#include "application/gui/panels/TrimPanel.hpp"
#include "application/sim/Aircraft.hpp"
#include "application/sim/Simulation.hpp"
#include "application/sim/control/FlightControlMode.hpp"
#include "application/sim/gnc/Autopilot.hpp"
#include "flightui/FlightUI.hpp"

#include <iostream>

namespace gui {
namespace UI = FlightUI;

namespace {
constexpr double DegToRad = 0.017453292519943295769;

gnc::RollHoldSettings MakeRollHoldSettings(const AutopilotPanelState &state) {
  return {
      .targetRollRad = state.rollTargetDeg * DegToRad,
      .dampingRatio = state.rollHoldDampingRatio,
      .naturalFrequencyRadPerSec = state.rollHoldNaturalFrequencyRadPerSec,
  };
}

gnc::PitchHoldSettings MakePitchHoldSettings(const AutopilotPanelState &state) {
  return {
      .targetPitchRad = state.pitchTargetDeg * DegToRad,
      .dampingRatio = state.pitchHoldDampingRatio,
      .naturalFrequencyRadPerSec = state.pitchHoldNaturalFrequencyRadPerSec,
  };
}

bool HasAnyAutopilotHoldEnabled(const AutopilotPanelState &state) {
  return state.rollHold || state.pitchHold || state.yawHold
         || state.altitudeHold || state.courseHold;
}
} // namespace

GNCWindow::GNCWindow() : Window("GNC") {}

void GNCWindow::RequestTrim(PendingTrimCommand command) {
  if (pendingTrimCommand_ != PendingTrimCommand::None || trimInProgress_) {
    return;
  }

  pendingTrimCommand_ = command;
}

void GNCWindow::ExecutePendingTrim(gui::GUI &gui) {
  const PendingTrimCommand command = pendingTrimCommand_;
  if (command == PendingTrimCommand::None || trimInProgress_) {
    return;
  }

  pendingTrimCommand_ = PendingTrimCommand::None;
  trimInProgress_ = true;

  auto &simulation = gui.GetSimulation();
  auto &executionControl = gui.GetSimulationExecutionControl();
  auto &aircraft = simulation.GetAircraft();
  const double simTime = aircraft.GetAircraftState().simulationTimeSec;
  const bool resumeAfterTrim =
      executionControl.GetSimulationExecutionState()
      == application::SimulationExecutionState::Running;
  executionControl.PauseSimulation();

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

  auto *flightControlManager =
      simulation.GetComponent<control::FlightControlManager>();
  if (flightControlManager == nullptr) {
    trimInProgress_ = false;
    if (resumeAfterTrim) {
      executionControl.ResumeSimulation();
    }
    return;
  }
  auto &autopilot = flightControlManager->GetAutopilot();
  bool trimSuccess = false;
  if (command == PendingTrimCommand::RunInitialCondition) {
    trimSuccess = autopilot.ComputeTrim(aircraft, trimRequest_);
  } else if (command == PendingTrimCommand::CurrentState) {
    trimSuccess =
        autopilot.ComputeCurrentStateTrim(aircraft, trimRequest_.mode);
  }

  if (trimSuccess && autopilot.ApplyStoredTrim(aircraft)) {
    if (const gnc::TrimResult *trimResult = autopilot.GetTrimResult()) {
      flightControlManager->SynchronizeWithTrimResult(*trimResult);
    }
  }

  trimResultOpen_ = true;
  trimResidualOpen_ = true;
  trimInProgress_ = false;

  if (resumeAfterTrim) {
    executionControl.ResumeSimulation();
  }
}

void GNCWindow::OnRender(gui::GUI &gui) {
  auto &simulation = gui.GetSimulation();
  auto &aircraft = simulation.GetAircraft();
  auto *flightControlManager =
      simulation.GetComponent<control::FlightControlManager>();
  if (flightControlManager == nullptr) {
    return;
  }
  auto &manualController = flightControlManager->GetManualController();
  auto &autopilot = flightControlManager->GetAutopilot();
  const gnc::TrimResult emptyTrimResult{};
  const gnc::TrimResult *trimResult = autopilot.GetTrimResult();
  const bool trimHasResult = trimResult != nullptr;

  // clang-format off
  UI::VerticalLayout()
      [
        +UI::TabGroup("GNC")
            [
              +UI::Tab("Trim")
                  [
                    UI::Custom([this,
                                   emptyTrimResult,
                                   trimResult,
                                   trimHasResult] {
                      TrimPanel::Draw({
                          trimRequest_,
                          trimHasResult ? *trimResult : emptyTrimResult,
                          trimHasResult,
                          trimResultOpen_,
                          trimResidualOpen_,
                          !trimInProgress_
                              && pendingTrimCommand_ == PendingTrimCommand::None,
                          [this] {
                            RequestTrim(PendingTrimCommand::RunInitialCondition);
                          },
                          [this] {
                            RequestTrim(PendingTrimCommand::CurrentState);
                          },
                      });
                    })
                  ]
              + UI::Tab("Autopilot")
                    [
                      UI::Custom([this,
                                     flightControlManager,
                                     &autopilot,
                                     &aircraft] {
                        const auto &properties = aircraft.GetProperties();
                        const bool autopilotMode =
                            flightControlManager->GetMode()
                            == control::FlightControlMode::Autopilot;
                        const bool rollHoldEnabled =
                            autopilot.IsRollHoldEnabled();
                        const bool pitchHoldEnabled =
                            autopilot.IsPitchHoldEnabled();
                        const bool rollDynamicsReady =
                            autopilot.GetRollDynamics().has_value();
                        const bool pitchDynamicsReady =
                            autopilot.GetPitchDynamics().has_value();
                        AutopilotPanel::Draw({
                            .state = autopilotPanelState_,
                            .currentRollDeg = properties.Roll().Deg(),
                            .currentRollRateDegPerSec =
                                properties.P().DegPerSec(),
                            .currentAileron =
                                aircraft.GetControls().GetAileron(),
                            .rollHoldActive =
                                autopilotMode && rollHoldEnabled
                                && rollDynamicsReady,
                            .rollHoldPreparing =
                                autopilotMode && rollHoldEnabled
                                && !rollDynamicsReady,
                            .captureCurrentRoll = [this, &properties] {
                              autopilotPanelState_.rollTargetDeg =
                                  properties.Roll().Deg();
                            },
                            .currentPitchDeg = properties.Pitch().Deg(),
                            .currentPitchRateDegPerSec =
                                properties.Q().DegPerSec(),
                            .currentElevator =
                                aircraft.GetControls().GetElevator(),
                            .pitchHoldActive =
                                autopilotMode && pitchHoldEnabled
                                && pitchDynamicsReady,
                            .pitchHoldPreparing =
                                autopilotMode && pitchHoldEnabled
                                && !pitchDynamicsReady,
                            .captureCurrentPitch = [this, &properties] {
                              autopilotPanelState_.pitchTargetDeg =
                                  properties.Pitch().Deg();
                            },
                        });
                      })
                    ]
              + UI::Tab("Flight Control")
                    [
                      UI::Custom([&manualController, &aircraft, this] {
                        ManualControlPanel::Draw(
                            manualController, aircraft, autopilotPanelState_);
                      })
                    ]
            ]
      ]
      .Render();
  // clang-format on

  autopilot.SetRollHoldEnabled(autopilotPanelState_.rollHold);
  autopilot.SetRollHoldSettings(MakeRollHoldSettings(autopilotPanelState_));
  autopilot.SetPitchHoldEnabled(autopilotPanelState_.pitchHold);
  autopilot.SetPitchHoldSettings(MakePitchHoldSettings(autopilotPanelState_));
  flightControlManager->SetMode(HasAnyAutopilotHoldEnabled(autopilotPanelState_)
                                    ? control::FlightControlMode::Autopilot
                                    : control::FlightControlMode::Manual);
  ExecutePendingTrim(gui);
}
} // namespace gui
