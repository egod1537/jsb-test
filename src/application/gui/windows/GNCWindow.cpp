#include "GNCWindow.hpp"

#include "application/gui/GUI.hpp"
#include "application/gui/panels/AutopilotPanel.hpp"
#include "application/gui/panels/ManualControlPanel.hpp"
#include "application/gui/panels/TrimPanel.hpp"
#include "application/sim/Aircraft.hpp"
#include "application/sim/Simulation.hpp"
#include "flightui/FlightUI.hpp"

#include <iostream>

namespace gui {
namespace UI = FlightUI;

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

  // clang-format off
  UI::VerticalLayout()
      [
        +UI::TabGroup("GNC")
            [
              +UI::Tab("Trim")
                  [
                    UI::Custom([this] {
                      TrimPanel::Draw({
                          trimRequest_,
                          trimResult_,
                          trimHasResult_,
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
                      UI::Custom([this] {
                        AutopilotPanel::Draw(autopilotPanelState_);
                      })
                    ]
              + UI::Tab("Flight Control")
                    [
                      UI::Custom([&aircraft, this] {
                        ManualControlPanel::Draw(
                            aircraft, autopilotPanelState_);
                      })
                    ]
            ]
      ]
      .Render();
  // clang-format on

  ExecutePendingTrim(simulation);
}
} // namespace gui
