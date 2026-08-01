#include "application/sim/gnc/hold/PitchHoldController.hpp"
#include "application/sim/Aircraft.hpp"
#include "application/sim/jsbsim/Properties.hpp"

namespace gnc {
void PitchHoldController::Reset() {}

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

std::optional<double> PitchHoldController::OnTick(const sim::Aircraft &aircraft,
    const sim::Tick &) {
  if (!enabled_) {
    return std::nullopt;
  }

  const sim::jsbsim::Properties &prop = aircraft.GetProperties();

  const double error = settings_.targetPitchRad - prop.Pitch().Rad();
  const double newElevator = GetTrimElevator()
                             - settings_.proportionalGain * error
                             + settings_.derivativeGain * prop.Q().RadPerSec();

  return newElevator;
}

} // namespace gnc
