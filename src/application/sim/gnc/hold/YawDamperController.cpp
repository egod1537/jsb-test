#include "application/sim/gnc/hold/YawDamperController.hpp"

namespace gnc {
void YawDamperController::Reset() {}

bool YawDamperController::IsEnabled() const { return enabled_; }

void YawDamperController::SetEnabled(bool enabled) { enabled_ = enabled; }

const YawDamperSettings &YawDamperController::GetSettings() const {
  return settings_;
}

void YawDamperController::SetSettings(const YawDamperSettings &settings) {
  settings_ = settings;
}

double YawDamperController::GetTrimRudder() const { return trimRudder_; }

void YawDamperController::SetTrimRudder(double trimRudder) {
  trimRudder_ = trimRudder;
}

std::optional<double> YawDamperController::OnTick(const sim::Aircraft &,
    const sim::Tick &) {
  // TODO: Implement yaw damping and return the commanded rudder input.
  return std::nullopt;
}
} // namespace gnc
