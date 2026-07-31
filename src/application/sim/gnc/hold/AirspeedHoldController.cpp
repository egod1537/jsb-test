#include "application/sim/gnc/hold/AirspeedHoldController.hpp"
#include "application/sim/Aircraft.hpp"
#include "application/sim/jsbsim/FlightProperties.hpp"

namespace gnc {
void AirspeedHoldController::Reset() {}

bool AirspeedHoldController::Reset(sim::Context &) {
  Reset();
  return true;
}

bool AirspeedHoldController::IsEnabled() const { return enabled_; }

void AirspeedHoldController::SetEnabled(bool enabled) { enabled_ = enabled; }

const AirspeedHoldSettings &AirspeedHoldController::GetSettings() const {
  return settings_;
}

void AirspeedHoldController::SetSettings(const AirspeedHoldSettings &settings) {
  settings_ = settings;
}

double AirspeedHoldController::GetTrimThrottle() const { return trimThrottle_; }

void AirspeedHoldController::SetTrimThrottle(double trimThrottle) {
  trimThrottle_ = trimThrottle;
}

std::optional<double> AirspeedHoldController::Update(
    const sim::Aircraft &aircraft, double) {
  if (!enabled_) {
    return std::nullopt;
  }

  const JSBSim::FlightProperties &prop = aircraft.GetProperties();

  const double error = settings_.targetAirspeedMps
                       - prop.TrueAirspeed().Mps();
  const double newThrottle = GetTrimThrottle()
                             + settings_.proportionalGain * error
                             - settings_.derivativeGain * prop.U().DotMps2();

  return newThrottle;
}

bool AirspeedHoldController::PreStep(sim::Context &, const sim::Tick &) {
  return true;
}
} // namespace gnc
