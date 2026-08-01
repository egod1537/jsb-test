#include "application/sim/gnc/hold/RollHoldController.hpp"

#include "application/sim/Aircraft.hpp"
#include "application/sim/Tick.hpp"

namespace gnc {
void RollHoldController::Reset() {}

bool RollHoldController::IsEnabled() const { return enabled_; }

void RollHoldController::SetEnabled(bool enabled) { enabled_ = enabled; }

const RollHoldSettings &RollHoldController::GetSettings() const {
  return settings_;
}

void RollHoldController::SetSettings(const RollHoldSettings &settings) {
  settings_ = settings;
}

double RollHoldController::GetTrimAileron() const { return trimAileron_; }

void RollHoldController::SetTrimAileron(double trimAileron) {
  trimAileron_ = trimAileron;
}

std::optional<double> RollHoldController::OnTick(const sim::Aircraft &aircraft,
    const sim::Tick &) {
  if (!enabled_) {
    return std::nullopt;
  }

  const auto &prop = aircraft.GetProperties();

  const double error = settings_.targetRollRad - prop.Roll().Rad();
  const double newAileron = GetTrimAileron()
                            + settings_.proportionalGain * error
                            - settings_.derivativeGain * prop.P().RadPerSec();

  return newAileron;
}

} // namespace gnc
