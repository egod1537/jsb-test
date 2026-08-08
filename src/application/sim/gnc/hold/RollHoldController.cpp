#include "application/sim/gnc/hold/RollHoldController.hpp"

#include "application/sim/Aircraft.hpp"
#include "application/sim/Tick.hpp"
#include "application/sim/gnc/ControlContext.hpp"

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
    const sim::Tick &, const ControlContext &context) {
  if (!enabled_) {
    return std::nullopt;
  }

  const auto &prop = aircraft.GetProperties();
  const auto &[aPhi1, aPhi2] = *context.rollDynamics;

  const double wN = settings_.naturalFrequencyRadPerSec;
  const double zeta = settings_.dampingRatio;

  const double kP = wN * wN / aPhi2;
  const double kD = (2 * zeta * wN - aPhi1) / aPhi2;

  const double error = settings_.targetRollRad - prop.Roll().Rad();
  const double newAileron =
      GetTrimAileron() + kP * error - kD * prop.P().RadPerSec();

  return newAileron;
}

} // namespace gnc
