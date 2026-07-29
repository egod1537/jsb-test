#include "application/Application.hpp"
#include "application/gui/GUI.hpp"
#include "application/gui/GUIConfig.hpp"
#include "application/sim/Simulation.hpp"
#include "application/sim/SimulationConfig.h"

#include <csignal>
#include <memory>

namespace {
volatile std::sig_atomic_t running = 1;
}

void HandleSignal(int) { running = 0; }

int main() {
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  sim::SimulationConfig simConfig;
  gui::GUIConfig guiConfig;
  std::unique_ptr<sim::Simulation> sim = std::make_unique<sim::Simulation>();
  std::unique_ptr<gui::GUI> gui =
      std::make_unique<gui::GUI>(sim.get(), guiConfig);

  Application app(std::move(gui), std::move(sim), simConfig);
  return app.Run(running) ? 0 : 1;
}
