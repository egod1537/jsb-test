#pragma once

#include "simulation/SimulationConfig.h"

#include <csignal>
#include <memory>

namespace sim {
class Simulation;
}
namespace gui {
class GUI;
}

class Application {
public:
  Application() = default;
  Application(std::unique_ptr<gui::GUI>, std::unique_ptr<sim::Simulation>,
      sim::SimulationConfig);

  bool Run(const volatile std::sig_atomic_t &);

private:
  bool Start();
  bool UpdateSimulation();
  void UpdateGUI();
  void Exit();

  std::unique_ptr<gui::GUI> gui_;
  std::unique_ptr<sim::Simulation> sim_;
  sim::SimulationConfig simConfig_;
};
