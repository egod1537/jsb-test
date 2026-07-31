#pragma once

#include "application/sim/System.hpp"
#include <optional>

namespace sim {
class Aircraft;
}

namespace gnc {
struct AirspeedHoldSettings {
  double targetAirspeedMps = 0.0;
  double proportionalGain = 0.5;
  double derivativeGain = 0.0;
};

class AirspeedHoldController final : public sim::System {
public:
  void Reset();
  bool Reset(sim::Context &context) override;

  bool IsEnabled() const;
  void SetEnabled(bool enabled);

  const AirspeedHoldSettings &GetSettings() const;
  void SetSettings(const AirspeedHoldSettings &settings);

  double GetTrimThrottle() const;
  void SetTrimThrottle(double trimThrottle);

  std::optional<double> Update(const sim::Aircraft &aircraft, double);
  bool PreStep(sim::Context &context, const sim::Tick &tick) override;

private:
  bool enabled_ = false;
  AirspeedHoldSettings settings_;
  double trimThrottle_ = 0.0;
};
} // namespace gnc
