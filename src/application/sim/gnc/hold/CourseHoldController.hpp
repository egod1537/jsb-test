#pragma once

#include "application/sim/gnc/Controller.hpp"

#include <optional>

namespace sim {
class Aircraft;
struct Tick;
} // namespace sim

namespace gnc {
struct ControlContext;

struct CourseHoldSettings {
  double targetCourseRad = 0.0;
  double dampingRatio = 0.7;
  double bandwidthSeparationRatio = 5.0;
};

class CourseHoldController final : public Controller {
public:
  void Reset() override;

  bool IsEnabled() const;
  void SetEnabled(bool enabled);

  const CourseHoldSettings &GetSettings() const;
  void SetSettings(const CourseHoldSettings &settings);

  std::optional<double> OnTick(const sim::Aircraft &aircraft,
      const sim::Tick &tick, const ControlContext &context);

private:
  bool enabled_ = false;
  CourseHoldSettings settings_;
  double integralCourseErrorRadSec_ = 0.0;
  double prevError_ = 0.0;
};
} // namespace gnc
