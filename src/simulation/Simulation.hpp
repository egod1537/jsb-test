#pragma once

#include "control/KeyboardInput.hpp"
#include "flightgear/FlightGearSender.hpp"
#include "simulation/SimulationConfig.h"
#include <memory>

namespace sim {
class FlightDynamics;

class Simulation {
public:
  Simulation();
  ~Simulation();

  Simulation(const Simulation &) = delete;
  Simulation &operator=(const Simulation &) = delete;

  bool Start(const SimulationConfig &);
  bool Update();
  void Exit();

  bool Initialize(const SimulationConfig &);

  const SimulationConfig &GetConfig() const { return config_; }

  FlightDynamics &GetFlightDynamics();
  const FlightDynamics &GetFlightDynamics() const;

private:
  void PrintState() const;

  std::unique_ptr<FlightDynamics> flightDynamics_;
  flightgear::FlightGearSender flightGearSender_;

  control::KeyboardInput keyboardInput_;

  SimulationConfig config_;
  double nextLogTime_ = 0.0;
  bool started_ = false;
};
} // namespace sim
