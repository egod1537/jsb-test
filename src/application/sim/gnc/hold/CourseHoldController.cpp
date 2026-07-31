#include "application/sim/gnc/hold/CourseHoldController.hpp"

namespace gnc {
void CourseHoldController::Reset() {}

bool CourseHoldController::Reset(sim::Context &) {
  Reset();
  return true;
}

bool CourseHoldController::IsEnabled() const { return enabled_; }

void CourseHoldController::SetEnabled(bool enabled) { enabled_ = enabled; }

const CourseHoldSettings &CourseHoldController::GetSettings() const {
  return settings_;
}

void CourseHoldController::SetSettings(const CourseHoldSettings &settings) {
  settings_ = settings;
}

double CourseHoldController::GetTrimAileron() const { return trimAileron_; }

void CourseHoldController::SetTrimAileron(double trimAileron) {
  trimAileron_ = trimAileron;
}

bool CourseHoldController::PreStep(sim::Context &, const sim::Tick &) {
  return true;
}
} // namespace gnc
