#include "application/sim/gnc/hold/CourseHoldController.hpp"
#include "application/sim/Aircraft.hpp"
#include "application/sim/Tick.hpp"
#include "application/sim/gnc/ControlContext.hpp"
#include "common/math/Math.hpp"

namespace gnc {
void CourseHoldController::Reset() { integralCourseErrorRadSec_ = 0.0; }

bool CourseHoldController::IsEnabled() const { return enabled_; }

void CourseHoldController::SetEnabled(bool enabled) { enabled_ = enabled; }

const CourseHoldSettings &CourseHoldController::GetSettings() const {
  return settings_;
}

void CourseHoldController::SetSettings(const CourseHoldSettings &settings) {
  settings_ = settings;
}

std::optional<double> CourseHoldController::OnTick(
    const sim::Aircraft &aircraft, const sim::Tick &tick,
    const ControlContext &context) {
  if (!enabled_) {
    return std::nullopt;
  }

  const auto &prop = aircraft.GetProperties();
  const double vG = prop.GroundSpeed().Mps();
  const double g = prop.GravityMps2();

  const double error =
      math::DeltaAngleRad(prop.Course().Rad(), settings_.targetCourseRad);
  integralCourseErrorRadSec_ += tick.dtSec * (error + prevError_) / 2.0f;

  const double rollWN = context.rollHoldSettings->naturalFrequencyRadPerSec;
  const double wN = rollWN / settings_.naturalFrequencyRadPerSec;
  const double zeta = settings_.dampingRatio;

  const double k0 = wN * vG / g;
  const double kP = 2 * zeta * k0;
  const double kI = wN * k0;
  const double newRoll = kP * error + kI * integralCourseErrorRadSec_;

  prevError_ = error;
  return newRoll;
}
} // namespace gnc
