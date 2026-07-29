#include "application/sim/Simulation.hpp"
#include "application/sim/Context.hpp"
#include "application/sim/gnc/TrimSolver.hpp"
#include "application/sim/SimulationConfig.h"
#include "application/sim/state/TrueStateProvider.hpp"
#include <FGFDMExec.h>
#include <cmath>
#include <iostream>
#include <utility>

namespace {
constexpr double LOG_INTERVAL_SEC = 1.0;

sim::InitialCondition InitialConditionFromConfig(
    const sim::SimulationConfig &config) {
  sim::InitialCondition initialCondition{};
  initialCondition.altitudeFt = config.altitudeFt;
  initialCondition.rollDeg = config.rollDeg;
  initialCondition.pitchDeg = config.pitchDeg;
  initialCondition.headingDeg = config.headingDeg;
  initialCondition.airspeedKts = config.calibratedAirspeedKts;
  return initialCondition;
}

double NormalizeHeadingDeg(double headingDeg) {
  double normalized = std::fmod(headingDeg, 360.0);
  if (normalized < 0.0) {
    normalized += 360.0;
  }

  return normalized;
}

gnc::TrimRequest TrimRequestFromInitialCondition(
    const sim::InitialCondition &initialCondition) {
  gnc::TrimRequest request{};
  request.mode = gnc::TrimMode::Full;
  request.airspeedKts = initialCondition.airspeedKts;
  request.altitudeFt = initialCondition.altitudeFt;
  request.flightPathAngleDeg = 0.0;
  return request;
}
} // namespace

namespace sim {
const char *ToString(SimulationState state) {
  switch (state) {
  case SimulationState::Running:
    return "Running";
  case SimulationState::Paused:
    return "Paused";
  case SimulationState::Stopped:
    return "Stopped";
  }

  return "Unknown";
}

// public
Simulation::Simulation()
    : stateProvider_(std::make_unique<state::TrueStateProvider>(aircraft_)) {
  RegisterSystems();
}

Simulation::~Simulation() = default;

bool Simulation::Start(const SimulationConfig &config) {
  if (started_) {
    return true;
  }

  config_ = config;
  defaultInitialCondition_ = InitialConditionFromConfig(config_);
  initialCondition_ = defaultInitialCondition_;
  ResetLogTimer();
  pendingSteps_ = 0;
  tickIndex_ = 0;
  lastError_.reset();

  if (!aircraft_.Initialize(config_)) {
    SetError("Failed to initialize aircraft.");
    state_ = SimulationState::Stopped;
    return false;
  }

  if (!ApplyInitialTrim(initialCondition_)) {
    state_ = SimulationState::Stopped;
    return false;
  }

  if (!InitializeSystems()) {
    if (!lastError_.has_value()) {
      SetError("Failed to initialize simulation systems.");
    }
    ShutdownSystems();
    state_ = SimulationState::Stopped;
    return false;
  }

  started_ = true;
  state_ = SimulationState::Running;
  return true;
}

bool Simulation::Update() {
  if (!started_) {
    return false;
  }

  if (state_ == SimulationState::Paused && pendingSteps_ == 0) {
    return true;
  }

  const bool shouldStep =
      state_ == SimulationState::Running
      || (state_ == SimulationState::Paused && pendingSteps_ > 0);
  if (!shouldStep) {
    return true;
  }

  if (!AdvanceOneTick()) {
    return false;
  }

  if (state_ == SimulationState::Paused && pendingSteps_ > 0) {
    --pendingSteps_;
  }

  return true;
}

void Simulation::Exit() {
  if (started_) {
    ShutdownSystems();
  }

  started_ = false;
  pendingSteps_ = 0;
  state_ = SimulationState::Stopped;
}

bool Simulation::Initialize(const SimulationConfig &config) {
  return Start(config);
}

void Simulation::Pause() {
  if (state_ == SimulationState::Running) {
    state_ = SimulationState::Paused;
  }
}

void Simulation::Resume() {
  if (state_ == SimulationState::Paused) {
    pendingSteps_ = 0;
    state_ = SimulationState::Running;
  }
}

void Simulation::TogglePause() {
  if (IsRunning()) {
    Pause();
  } else if (IsPaused()) {
    Resume();
  }
}

bool Simulation::RequestStep() {
  if (!IsPaused()) {
    return false;
  }

  ++pendingSteps_;
  return true;
}

bool Simulation::StepOnce() {
  if (!started_ || !IsPaused()) {
    return false;
  }

  return AdvanceOneTick();
}

bool Simulation::SetInitialCondition(const InitialCondition &initialCondition) {
  InitialCondition normalized = initialCondition;
  normalized.headingDeg = NormalizeHeadingDeg(normalized.headingDeg);

  if (!ValidateInitialCondition(normalized)) {
    return false;
  }

  initialCondition_ = normalized;
  lastError_.reset();
  return true;
}

bool Simulation::Restart() { return Restart(initialCondition_); }

bool Simulation::Restart(const InitialCondition &initialCondition) {
  if (!started_) {
    SetError("Simulation has not been started.");
    return false;
  }

  const SimulationState previousState = state_;
  pendingSteps_ = 0;

  if (!SetInitialCondition(initialCondition)) {
    state_ = SimulationState::Paused;
    return false;
  }

  if (!aircraft_.Reset(config_, initialCondition_)) {
    SetError(
        "Failed to restart aircraft with the requested initial condition.");
    state_ = SimulationState::Paused;
    return false;
  }

  if (!ApplyInitialTrim(initialCondition_)) {
    state_ = SimulationState::Paused;
    return false;
  }

  ResetLogTimer();
  tickIndex_ = 0;

  if (!ResetSystems()) {
    if (!lastError_.has_value()) {
      SetError("Failed to reset simulation systems.");
    }
    state_ = SimulationState::Paused;
    return false;
  }

  state_ = previousState == SimulationState::Paused ? SimulationState::Paused
                                                    : SimulationState::Running;
  lastError_.reset();
  return true;
}

InitialCondition Simulation::CaptureCurrentCondition() const {
  return aircraft_.CaptureCurrentCondition();
}

Aircraft &Simulation::GetAircraft() { return aircraft_; }

const Aircraft &Simulation::GetAircraft() const { return aircraft_; }

const state::IStateProvider &Simulation::GetStateProvider() const {
  return *stateProvider_;
}

bool Simulation::AdvanceOneTick() {
  const Tick preStepTick = MakeTick();
  if (!RunPreStepSystems(preStepTick)) {
    if (!lastError_.has_value()) {
      SetError("Simulation pre-step system failed.");
    }
    return false;
  }

  if (!aircraft_.Step()) {
    SetError("JSBSim simulation stopped.");
    std::cerr << *lastError_ << '\n';
    return false;
  }

  const Tick postStepTick = MakeTick();
  if (!RunPostStepSystems(postStepTick)) {
    if (!lastError_.has_value()) {
      SetError("Simulation post-step system failed.");
    }
    return false;
  }

  ++tickIndex_;

  const AircraftState aircraftState = aircraft_.GetAircraftState();
  if (aircraftState.simulationTimeSec >= nextLogTime_) {
    PrintState();
    nextLogTime_ += LOG_INTERVAL_SEC;
  }

  return true;
}

void Simulation::ResetLogTimer() { nextLogTime_ = 0.0; }

void Simulation::PrintState() const {
  const AircraftState state = aircraft_.GetAircraftState();
  std::cout << "t=" << state.simulationTimeSec
            << " s, altitude=" << state.altitudeAglFt
            << " ft, airspeed=" << state.calibratedAirspeedKts
            << " kt, pitch=" << state.pitchDeg << " deg\n";
}

bool Simulation::ValidateInitialCondition(
    const InitialCondition &initialCondition) {
  if (!std::isfinite(initialCondition.latitudeDeg)
      || initialCondition.latitudeDeg < -90.0
      || initialCondition.latitudeDeg > 90.0) {
    SetError("Latitude must be finite and between -90 and 90 degrees.");
    return false;
  }

  if (!std::isfinite(initialCondition.longitudeDeg)
      || initialCondition.longitudeDeg < -180.0
      || initialCondition.longitudeDeg > 180.0) {
    SetError("Longitude must be finite and between -180 and 180 degrees.");
    return false;
  }

  if (!std::isfinite(initialCondition.altitudeFt)) {
    SetError("Altitude must be finite.");
    return false;
  }

  if (!std::isfinite(initialCondition.rollDeg)
      || !std::isfinite(initialCondition.pitchDeg)
      || !std::isfinite(initialCondition.headingDeg)) {
    SetError("Attitude values must be finite.");
    return false;
  }

  if (!std::isfinite(initialCondition.airspeedKts)
      || initialCondition.airspeedKts < 0.0) {
    SetError("Airspeed must be finite and non-negative.");
    return false;
  }

  if (!std::isfinite(initialCondition.pRadPerSec)
      || !std::isfinite(initialCondition.qRadPerSec)
      || !std::isfinite(initialCondition.rRadPerSec)) {
    SetError("Angular rates must be finite.");
    return false;
  }

  return true;
}

Context Simulation::MakeContext() {
  return Context(aircraft_, config_, &lastError_);
}

bool Simulation::InitializeSystems() {
  Context context = MakeContext();
  for (System *system : systems_) {
    if (system != nullptr && !system->Initialize(context)) {
      return false;
    }
  }

  return true;
}

bool Simulation::ResetSystems() {
  Context context = MakeContext();
  for (System *system : systems_) {
    if (system != nullptr && !system->Reset(context)) {
      return false;
    }
  }

  return true;
}

bool Simulation::RunPreStepSystems(const Tick &tick) {
  Context context = MakeContext();
  for (System *system : systems_) {
    if (system != nullptr && !system->PreStep(context, tick)) {
      return false;
    }
  }

  return true;
}

bool Simulation::RunPostStepSystems(const Tick &tick) {
  Context context = MakeContext();
  for (System *system : systems_) {
    if (system != nullptr && !system->PostStep(context, tick)) {
      return false;
    }
  }

  return true;
}

void Simulation::ShutdownSystems() {
  Context context = MakeContext();
  for (auto iter = systems_.rbegin(); iter != systems_.rend(); ++iter) {
    if (*iter != nullptr) {
      (*iter)->Shutdown(context);
    }
  }
}

void Simulation::RegisterSystems() {
  systems_.clear();
  systems_.push_back(&keyboard_);
  systems_.push_back(&rollHold_);
  systems_.push_back(&flightGear_);
}

bool Simulation::ApplyInitialTrim(const InitialCondition &initialCondition) {
  gnc::TrimSolver trimSolver;
  const gnc::TrimResult trimResult =
      trimSolver.Trim(aircraft_,
          TrimRequestFromInitialCondition(initialCondition));

  if (!trimResult.success) {
    SetError(trimResult.message.empty() ? "Initial trim failed."
                                        : trimResult.message);
    std::cerr << "Initial trim failed: " << *lastError_ << '\n';
    return false;
  }

  aircraft_.GetFDMExec().Setsim_time(0.0);
  return true;
}

Tick Simulation::MakeTick() const {
  return Tick{tickIndex_,
      config_.GetDT(),
      aircraft_.GetAircraftState().simulationTimeSec};
}

void Simulation::SetError(std::string message) {
  lastError_ = std::move(message);
}
} // namespace sim
