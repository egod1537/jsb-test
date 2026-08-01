#pragma once

#include "application/SimulationExecutionControl.hpp"
#include "application/flightgear/FlightGearSystem.hpp"
#include "application/sim/SimulationConfig.h"

#include <csignal>
#include <cstdint>
#include <memory>

namespace sim {
class Simulation;
}
namespace gui {
class GUI;
}

class Application : public application::SimulationExecutionControl {
public:
  // Lifetime and main loop
  Application() = default;
  ~Application();
  Application(std::unique_ptr<gui::GUI>, std::unique_ptr<sim::Simulation>,
      sim::SimulationConfig);
  bool Run(const volatile std::sig_atomic_t &);

  // Simulation execution control
  application::SimulationExecutionState
  GetSimulationExecutionState() const override {
    return simulationExecutionState_;
  }
  void PauseSimulation() override;
  void ResumeSimulation() override;
  void RequestSimulationTick() override;
  bool ResetSimulation() override;
  bool ResetSimulation(const sim::InitialCondition &initialCondition) override;
  std::uint32_t GetPendingSimulationTickCount() const override {
    return pendingSimulationTicks_;
  }

private:
  // Application lifecycle
  bool Start();
  bool TickSimulation();
  void TickGUI();
  void Exit();

  // Owned services
  std::unique_ptr<sim::Simulation> sim_;
  std::unique_ptr<gui::GUI> gui_;
  flightgear::FlightGearSystem flightGear_;

  // Configuration
  sim::SimulationConfig simConfig_;

  // Execution state
  application::SimulationExecutionState simulationExecutionState_ =
      application::SimulationExecutionState::Stopped;
  std::uint32_t pendingSimulationTicks_ = 0;
};
