#include "application/sim/gnc/hold/PitchHoldController.hpp"
#include "application/sim/Aircraft.hpp"
#include "application/sim/jsbsim/FlightProperties.hpp"

namespace gnc {
void PitchHoldController::Reset() {}

bool PitchHoldController::Reset(sim::Context &) {
  Reset();
  return true;
}

bool PitchHoldController::IsEnabled() const { return enabled_; }

void PitchHoldController::SetEnabled(bool enabled) { enabled_ = enabled; }

const PitchHoldSettings &PitchHoldController::GetSettings() const {
  return settings_;
}

void PitchHoldController::SetSettings(const PitchHoldSettings &settings) {
  settings_ = settings;
}

double PitchHoldController::GetTrimElevator() const { return trimElevator_; }

void PitchHoldController::SetTrimElevator(double trimElevator) {
  trimElevator_ = trimElevator;
}

std::optional<double> PitchHoldController::Update(const sim::Aircraft &aircraft,
    double) {
  if (!enabled_) {
    return std::nullopt;
  }

  const JSBSim::FlightProperties &prop = aircraft.GetProperties();

  const double error = settings_.targetPitchRad - prop.Pitch().Rad();
  const double newElevator = GetTrimElevator()
                             - settings_.proportionalGain * error
                             + settings_.derivativeGain * prop.Q().RadPerSec();

  return newElevator;
}

bool PitchHoldController::PreStep(sim::Context &, const sim::Tick &) {
  return true;
}
} // namespace gnc
