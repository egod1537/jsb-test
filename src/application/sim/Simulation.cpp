#include "application/sim/Simulation.hpp"
#include "application/sim/SimulationConfig.h"
#include "application/sim/StateLogger.hpp"
#include <cmath>
#include <iostream>

namespace {
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
// public
Simulation::Simulation() {
  AddComponent<control::FlightControlManager>();
  AddComponent<StateLogger>();
}

Simulation::~Simulation() = default;

bool Simulation::Initialize(const SimulationConfig &config) {
  if (initialized_) {
    return true;
  }

  config_ = config;
  defaultInitialCondition_ = InitialCondition{};
  tickIndex_ = 0;
  errorTracker_.ClearError();

  auto *flightControlManager = GetComponent<control::FlightControlManager>();
  if (flightControlManager == nullptr) {
    errorTracker_.SetError("Flight control component is missing.");
    return false;
  }
  flightControlManager->ResetControllers();

  if (!aircraft_.Initialize(config_, defaultInitialCondition_)) {
    errorTracker_.SetError("Failed to initialize aircraft.");
    return false;
  }

  if (!ApplyInitialTrim(defaultInitialCondition_)) {
    return false;
  }

  if (!InitializeComponents()) {
    errorTracker_.SetErrorIfEmpty("Failed to initialize components.");
    ShutdownComponents();
    return false;
  }

  initialized_ = true;
  return true;
}

bool Simulation::Tick() {
  if (!initialized_) {
    return false;
  }

  return ProcessTick();
}

void Simulation::Shutdown() {
  if (initialized_) {
    ShutdownComponents();
  }

  initialized_ = false;
}

const SimulationConfig &Simulation::GetConfig() const { return config_; }

double Simulation::GetTickSizeSec() const { return config_.GetDT(); }

bool Simulation::Reset() { return Reset(defaultInitialCondition_); }

bool Simulation::Reset(const InitialCondition &initialCondition) {
  if (!initialized_) {
    errorTracker_.SetError("Simulation has not been initialized.");
    return false;
  }

  InitialCondition normalized = initialCondition;
  normalized.headingDeg = NormalizeHeadingDeg(normalized.headingDeg);

  std::string validationError;
  if (!ValidateInitialCondition(normalized, &validationError)) {
    errorTracker_.SetError(validationError);
    return false;
  }

  auto *flightControlManager = GetComponent<control::FlightControlManager>();
  if (flightControlManager == nullptr) {
    errorTracker_.SetError("Flight control component is missing.");
    return false;
  }

  if (!aircraft_.Reset(config_, normalized)) {
    errorTracker_.SetError(
        "Failed to reset aircraft with the requested initial condition.");
    return false;
  }

  flightControlManager->ResetControllers();

  if (!ApplyInitialTrim(normalized)) {
    return false;
  }

  tickIndex_ = 0;

  if (!ResetComponents()) {
    errorTracker_.SetErrorIfEmpty("Failed to reset components.");
    return false;
  }

  errorTracker_.ClearError();
  return true;
}

InitialCondition Simulation::GetCurrentCondition() const {
  return aircraft_.GetCurrentCondition();
}

const InitialCondition &Simulation::GetDefaultInitialCondition() const {
  return defaultInitialCondition_;
}

Aircraft &Simulation::GetAircraft() { return aircraft_; }

const Aircraft &Simulation::GetAircraft() const { return aircraft_; }

ErrorTracker &Simulation::GetErrorTracker() { return errorTracker_; }

const ErrorTracker &Simulation::GetErrorTracker() const {
  return errorTracker_;
}

bool Simulation::ProcessTick() {
  const sim::Tick tick = MakeTick();

  if (!RunPreTickComponents(tick)) {
    errorTracker_.SetErrorIfEmpty("Simulation pre-tick component failed.");
    return false;
  }

  auto *flightControlManager = GetComponent<control::FlightControlManager>();
  if (flightControlManager == nullptr) {
    errorTracker_.SetError("Flight control component is missing.");
    return false;
  }

  if (!TickComponents(tick)) {
    errorTracker_.SetErrorIfEmpty("Simulation tick component failed.");
    return false;
  }

  if (!aircraft_.Tick()) {
    errorTracker_.SetError("JSBSim simulation stopped.");
    std::cerr << errorTracker_.GetLastError().value() << '\n';
    return false;
  }

  const sim::Tick postTick = MakeTick();
  if (!RunPostTickComponents(postTick)) {
    errorTracker_.SetErrorIfEmpty("Simulation post-tick component failed.");
    return false;
  }

  ++tickIndex_;

  return true;
}

sim::Tick Simulation::MakeTick() const {
  return sim::Tick{tickIndex_,
      config_.GetDT(),
      aircraft_.GetAircraftState().simulationTimeSec};
}

bool Simulation::ApplyInitialTrim(const InitialCondition &initialCondition) {
  auto *flightControlManager = GetComponent<control::FlightControlManager>();
  if (flightControlManager == nullptr) {
    errorTracker_.SetError("Flight control component is missing.");
    return false;
  }

  gnc::Autopilot &autopilot = flightControlManager->GetAutopilot();
  if (!autopilot.ComputeTrim(aircraft_,
          TrimRequestFromInitialCondition(initialCondition))) {
    errorTracker_.SetError("Initial trim failed.");
    std::cerr << "Initial trim failed: " << errorTracker_.GetLastError().value()
              << '\n';
    return false;
  }

  if (!autopilot.ApplyStoredTrim(aircraft_)) {
    errorTracker_.SetError("Failed to apply stored initial trim.");
    std::cerr << errorTracker_.GetLastError().value() << '\n';
    return false;
  }

  if (const gnc::TrimResult *trimResult = autopilot.GetTrimResult()) {
    flightControlManager->SynchronizeWithTrimResult(*trimResult);
  }

  aircraft_.ResetSimulationTime();
  return true;
}

bool Simulation::InitializeComponent(Component &component) {
  if (component.initialized_) {
    return true;
  }

  if (!component.OnInitialize()) {
    component.OnShutdown();
    return false;
  }

  component.initialized_ = true;
  return true;
}

bool Simulation::InitializeComponents() {
  for (std::size_t index = 0; index < components_.size(); ++index) {
    if (!InitializeComponent(*components_[index])) {
      return false;
    }
  }

  return true;
}

bool Simulation::ResetComponents() {
  for (const auto &component : components_) {
    if (component->initialized_ && !component->OnReset()) {
      return false;
    }
  }

  return true;
}

bool Simulation::RunPreTickComponents(const sim::Tick &tick) {
  for (const auto &component : components_) {
    if (component->initialized_ && !component->OnPreTick(tick)) {
      return false;
    }
  }

  return true;
}

bool Simulation::TickComponents(const sim::Tick &tick) {
  for (const auto &component : components_) {
    if (component->initialized_ && !component->OnTick(tick)) {
      return false;
    }
  }

  return true;
}

bool Simulation::RunPostTickComponents(const sim::Tick &tick) {
  for (const auto &component : components_) {
    if (component->initialized_ && !component->OnPostTick(tick)) {
      return false;
    }
  }

  return true;
}

void Simulation::ShutdownComponents() {
  for (auto iterator = components_.rbegin(); iterator != components_.rend();
      ++iterator) {
    if ((*iterator)->initialized_) {
      (*iterator)->OnShutdown();
      (*iterator)->initialized_ = false;
    }
  }
}

Component *Simulation::FindComponent(const std::type_info &type) {
  for (const auto &component : components_) {
    if (typeid(*component) == type) {
      return component.get();
    }
  }

  return nullptr;
}

const Component *Simulation::FindComponent(const std::type_info &type) const {
  for (const auto &component : components_) {
    if (typeid(*component) == type) {
      return component.get();
    }
  }

  return nullptr;
}

} // namespace sim
