#include "Application.hpp"
#include "gui/GUI.hpp"
#include "simulation/Simulation.hpp"

#include <algorithm>
#include <chrono>
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
} // namespace

Application::Application(std::unique_ptr<gui::GUI> gui,
    std::unique_ptr<sim::Simulation> sim, sim::SimulationConfig simConfig)
    : gui_(std::move(gui)), sim_(std::move(sim)),
      simConfig_(std::move(simConfig)) {}

bool Application::Run(const volatile std::sig_atomic_t &running) {
  if (!Start()) {
    Exit();
    return false;
  }

  bool succeeded = true;
  const Clock::duration simulationInterval =
      ToClockDuration(sim_->GetConfig().GetDT());
  const double guiDt = gui_->GetConfig().GetRenderDT();
  const Clock::duration guiInterval =
      guiDt > 0.0 ? ToClockDuration(guiDt) : simulationInterval;

  auto nextSimulationUpdate = Clock::now();
  auto nextGUIUpdate = nextSimulationUpdate;

  while (succeeded && running && !gui_->ShouldClose()) {
    auto now = Clock::now();

    while (now >= nextSimulationUpdate) {
      if (!UpdateSimulation()) {
        succeeded = false;
        break;
      }

      nextSimulationUpdate += simulationInterval;
      now = Clock::now();
    }

    if (!succeeded) {
      break;
    }

    if (now >= nextGUIUpdate) {
      UpdateGUI();
      do {
        nextGUIUpdate += guiInterval;
      } while (nextGUIUpdate <= now);
    }

    std::this_thread::sleep_until(
        std::min(nextSimulationUpdate, nextGUIUpdate));
  }

  Exit();
  return succeeded;
}

bool Application::Start() {
  if (gui_ == nullptr || sim_ == nullptr) {
    std::cerr << "Application requires GUI and simulation instances\n";
    return false;
  }

  if (!sim_->Start(simConfig_)) {
    std::cerr << "Failed to start simulation\n";
    return false;
  }

  if (!gui_->Start()) {
    std::cerr << "Failed to start GUI\n";
    return false;
  }

  return true;
}

bool Application::UpdateSimulation() { return sim_->Update(); }

void Application::UpdateGUI() { gui_->Update(); }

void Application::Exit() {
  if (gui_ != nullptr) {
    gui_->Exit();
  }

  if (sim_ != nullptr) {
    sim_->Exit();
  }
}
