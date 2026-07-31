#pragma once

#include "application/sim/System.hpp"

namespace gnc {
struct CourseHoldSettings {
  double targetCourseRad = 0.0;
  double proportionalGain = 0.5;
  double derivativeGain = 2.0;
};

class CourseHoldController final : public sim::System {
public:
  void Reset();
  bool Reset(sim::Context &context) override;

  bool IsEnabled() const;
  void SetEnabled(bool enabled);

  const CourseHoldSettings &GetSettings() const;
  void SetSettings(const CourseHoldSettings &settings);

  double GetTrimAileron() const;
  void SetTrimAileron(double trimAileron);

  bool PreStep(sim::Context &context, const sim::Tick &tick) override;

private:
  bool enabled_ = false;
  CourseHoldSettings settings_;
  double trimAileron_ = 0.0;
};
} // namespace gnc
