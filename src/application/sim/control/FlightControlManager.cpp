#include "application/sim/control/FlightControlManager.hpp"

#include "application/sim/Aircraft.hpp"

namespace control {
FlightControlManager::FlightControlManager() : autopilot_(manualController_) {}

bool FlightControlManager::OnTick(const sim::Tick &tick) {
  sim::Aircraft &aircraft = GetAircraft();
  autopilot_.UpdateLinearization(aircraft, tick);
  if (const auto input = ProduceControlInput(aircraft, tick)) {
    aircraft.GetControls().SetInput(*input);
  }

  return true;
}

std::optional<ControlInput> FlightControlManager::ProduceControlInput(
    sim::Aircraft &aircraft, const sim::Tick &tick) {
  switch (mode_) {
  case FlightControlMode::None:
    return std::nullopt;
  case FlightControlMode::Manual:
    return manualController_.OnTick(aircraft, tick);
  case FlightControlMode::Autopilot:
    return autopilot_.OnTick(aircraft, tick);
  }

  return std::nullopt;
}

FlightControlMode FlightControlManager::GetMode() const { return mode_; }

void FlightControlManager::SetMode(FlightControlMode mode) { mode_ = mode; }

ManualFlightControlController &FlightControlManager::GetManualController() {
  return manualController_;
}

const ManualFlightControlController &
FlightControlManager::GetManualController() const {
  return manualController_;
}

gnc::Autopilot &FlightControlManager::GetAutopilot() { return autopilot_; }

const gnc::Autopilot &FlightControlManager::GetAutopilot() const {
  return autopilot_;
}

void FlightControlManager::ResetControllers() {
  mode_ = FlightControlMode::Manual;
  manualController_.OnReset();
  autopilot_.OnReset();
}

void FlightControlManager::SynchronizeWithTrimResult(
    const gnc::TrimResult &trimResult) {
  manualController_.SetCommandedInput({
      .elevator = trimResult.elevator,
      .aileron = trimResult.aileron,
      .rudder = trimResult.rudder,
      .throttle = trimResult.throttle,
  });
}

} // namespace control
