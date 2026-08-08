#include "Application.hpp"
#include "application/gui/GUI.hpp"
#include "application/input/Input.hpp"
#include "application/sim/Simulation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>
#include <utility>

namespace {
using Clock = std::chrono::steady_clock;

Clock::duration ToClockDuration(double seconds) {
  return std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(seconds));
}

double ClampAutomaticSimulationHz(double hz) {
  if (!std::isfinite(hz)) {
    return application::MinimumAutomaticSimulationHz;
  }

  return std::clamp(hz,
      application::MinimumAutomaticSimulationHz,
      application::MaximumAutomaticSimulationHz);
}

Clock::duration ToSimulationInterval(double hz) {
  return ToClockDuration(1.0 / ClampAutomaticSimulationHz(hz));
}
} // namespace

Application::Application(std::unique_ptr<gui::GUI> gui,
    std::unique_ptr<sim::Simulation> sim, sim::SimulationConfig simConfig)
    : sim_(std::move(sim)), gui_(std::move(gui)),
      simConfig_(std::move(simConfig)),
      automaticSimulationHz_(
          ClampAutomaticSimulationHz(simConfig_.simulationHz)) {}

Application::~Application() = default;

bool Application::Run(const volatile std::sig_atomic_t &running) {
  if (!Start()) {
    Exit();
    return false;
  }

  bool succeeded = true;
  double scheduledSimulationHz = automaticSimulationHz_;
  Clock::duration simulationInterval =
      ToSimulationInterval(scheduledSimulationHz);
  const double guiDt = gui_->GetConfig().GetRenderDT();
  const Clock::duration guiInterval =
      guiDt > 0.0 ? ToClockDuration(guiDt) : simulationInterval;

  auto nextSimulationTick = Clock::now();
  auto nextGUITick = nextSimulationTick;

  while (succeeded && running && !gui_->ShouldClose()) {
    auto now = Clock::now();

    if (automaticSimulationHz_ != scheduledSimulationHz) {
      scheduledSimulationHz = automaticSimulationHz_;
      simulationInterval = ToSimulationInterval(scheduledSimulationHz);
      nextSimulationTick = now + simulationInterval;
    }

    const bool hasPendingManualTick =
        simulationExecutionState_
            == application::SimulationExecutionState::Paused
        && pendingSimulationTicks_ > 0;

    if (hasPendingManualTick) {
      if (!TickSimulation()) {
        succeeded = false;
      }
    } else {
      while (now >= nextSimulationTick) {
        if (!TickSimulation()) {
          succeeded = false;
          break;
        }

        nextSimulationTick += simulationInterval;
        now = Clock::now();
      }
    }

    if (!succeeded) {
      break;
    }

    if (now >= nextGUITick) {
      TickGUI();
      do {
        nextGUITick += guiInterval;
      } while (nextGUITick <= now);
    }

    std::this_thread::sleep_until(std::min(nextSimulationTick, nextGUITick));
  }

  Exit();
  return succeeded;
}

bool Application::Start() {
  if (gui_ == nullptr || sim_ == nullptr) {
    std::cerr << "Application requires GUI and simulation instances\n";
    return false;
  }

  gui_->SetSimulationExecutionControl(this);

  if (!application::Input::Initialize()) {
    std::cerr << "Failed to initialize input\n";
    return false;
  }

  if (!sim_->Initialize(simConfig_)) {
    std::cerr << "Failed to initialize simulation\n";
    return false;
  }

  if (!flightGear_.Initialize()) {
    return false;
  }

  if (!gui_->Start()) {
    std::cerr << "Failed to start GUI\n";
    return false;
  }

  simulationExecutionState_ = application::SimulationExecutionState::Running;
  pendingSimulationTicks_ = 0;
  return true;
}

bool Application::TickSimulation() {
  const bool isPaused = simulationExecutionState_
                        == application::SimulationExecutionState::Paused;
  if (isPaused && pendingSimulationTicks_ == 0) {
    return true;
  }

  if (simulationExecutionState_
          != application::SimulationExecutionState::Running
      && !isPaused) {
    return true;
  }

  application::Input::Update();

  if (!sim_->Tick()) {
    return false;
  }

  flightGear_.Update(sim_->GetAircraft());

  if (isPaused) {
    --pendingSimulationTicks_;
  }

  return true;
}

void Application::PauseSimulation() {
  if (simulationExecutionState_
      == application::SimulationExecutionState::Running) {
    simulationExecutionState_ = application::SimulationExecutionState::Paused;
  }
}

void Application::ResumeSimulation() {
  if (simulationExecutionState_
      == application::SimulationExecutionState::Paused) {
    pendingSimulationTicks_ = 0;
    simulationExecutionState_ = application::SimulationExecutionState::Running;
  }
}

void Application::RequestSimulationTick() {
  if (simulationExecutionState_
      == application::SimulationExecutionState::Paused) {
    ++pendingSimulationTicks_;
  }
}

void Application::SetAutomaticSimulationHz(double hz) {
  if (!std::isfinite(hz)) {
    return;
  }

  automaticSimulationHz_ = ClampAutomaticSimulationHz(hz);
}

bool Application::ResetSimulation() {
  if (!sim_->Reset()) {
    return false;
  }

  flightGear_.Update(sim_->GetAircraft());
  return true;
}

bool Application::ResetSimulation(
    const sim::InitialCondition &initialCondition) {
  if (!sim_->Reset(initialCondition)) {
    return false;
  }

  flightGear_.Update(sim_->GetAircraft());
  return true;
}

void Application::TickGUI() { gui_->Tick(); }

void Application::Exit() {
  simulationExecutionState_ = application::SimulationExecutionState::Stopped;
  pendingSimulationTicks_ = 0;

  if (gui_ != nullptr) {
    gui_->Exit();
  }

  flightGear_.Shutdown();

  if (sim_ != nullptr) {
    sim_->Shutdown();
  }

  application::Input::Shutdown();
}
