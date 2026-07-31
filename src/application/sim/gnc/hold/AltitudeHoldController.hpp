#pragma once

#include "application/sim/System.hpp"

namespace gnc {
struct AltitudeHoldSettings {
  double targetAltitudeFt = 0.0;
  double proportionalGain = 0.5;
  double derivativeGain = 0.0;
};

class AltitudeHoldController final : public sim::System {
public:
  void Reset();
  bool Reset(sim::Context &context) override;

  bool IsEnabled() const;
  void SetEnabled(bool enabled);

  const AltitudeHoldSettings &GetSettings() const;
  void SetSettings(const AltitudeHoldSettings &settings);

  double GetTrimElevator() const;
  void SetTrimElevator(double trimElevator);

  bool PreStep(sim::Context &context, const sim::Tick &tick) override;

private:
  bool enabled_ = false;
  AltitudeHoldSettings settings_;
  double trimElevator_ = 0.0;
};
} // namespace gnc
