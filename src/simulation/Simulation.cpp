#include "simulation/Simulation.hpp"
#include "simulation/FlightDynamics.hpp"
#include "simulation/SimulationConfig.h"
#include <iostream>

namespace {
constexpr double LOG_INTERVAL_SEC = 1.0;
} // namespace

namespace sim {
// public
Simulation::Simulation()
    : flightDynamics_(std::make_unique<FlightDynamics>()) {}
Simulation::~Simulation() = default;

bool Simulation::Start(const SimulationConfig &config) {
  if (started_) {
    return true;
  }

  config_ = config;
  nextLogTime_ = 0.0;

  if (!flightDynamics_->Initialize(config_)) {
    return false;
  }

  if (!flightGearSender_.IsOpen()) {
    std::cerr << "Failed to initialize FlightGear sender\n";
    return false;
  }

  if (!keyboardInput_.Initialize()) {
    std::cerr << "Failed to initialize keyboard input\n";
    return false;
  }

  started_ = true;
  return true;
}

bool Simulation::Update() {
  if (!started_) {
    return false;
  }

  control::ControlInput &controlInput = flightDynamics_->GetControlInput();
  if (keyboardInput_.Update(controlInput)) {
    std::cout << "control" << " elevator=" << controlInput.elevator
              << " aileron=" << controlInput.aileron
              << " rudder=" << controlInput.rudder
              << " throttle=" << controlInput.throttle << '\n';
  }

  if (!flightDynamics_->Update()) {
    std::cerr << "JSBSim simulation stopped\n";
    return false;
  }

  if (!flightGearSender_.Send(flightDynamics_->GetFDMExec())) {
    std::cerr << "Failed to send FlightGear packet\n";
  }

  const double simTime = flightDynamics_->GetProperties().GetSimTimeSec();
  if (simTime >= nextLogTime_) {
    PrintState();
    nextLogTime_ += LOG_INTERVAL_SEC;
  }

  return true;
}

void Simulation::Exit() { started_ = false; }

bool Simulation::Initialize(const SimulationConfig &config) {
  return Start(config);
}

FlightDynamics &Simulation::GetFlightDynamics() { return *flightDynamics_; }

const FlightDynamics &Simulation::GetFlightDynamics() const {
  return *flightDynamics_;
}

void Simulation::PrintState() const {
  const JSBSim::FlightProperties &properties =
      flightDynamics_->GetProperties();
  const double simTime = properties.GetSimTimeSec();
  const double altitude = properties.GetAltitudeAglFt();
  const double airspeed = properties.GetCalibratedAirspeedKts();
  const double pitch = properties.GetPitchRad();

  std::cout << "t=" << simTime << " s, altitude=" << altitude
            << " ft, airspeed=" << airspeed << " kt, pitch=" << pitch
            << " rad\n";
}
} // namespace sim
