#pragma once

#include "application/sim/System.hpp"
#include <optional>

namespace sim {
class Aircraft;
}

namespace gnc {
struct PitchHoldSettings {
  double targetPitchRad = 0.0;
  double proportionalGain = 0.5;
  double derivativeGain = 2.0;
};

class PitchHoldController final : public sim::System {
public:
  void Reset();
  bool Reset(sim::Context &context) override;

  bool IsEnabled() const;
  void SetEnabled(bool enabled);

  const PitchHoldSettings &GetSettings() const;
  void SetSettings(const PitchHoldSettings &settings);

  double GetTrimElevator() const;
  void SetTrimElevator(double trimElevator);

  std::optional<double> Update(const sim::Aircraft &aircraft, double);
  bool PreStep(sim::Context &context, const sim::Tick &tick) override;

private:
  bool enabled_ = false;
  PitchHoldSettings settings_;
  double trimElevator_ = 0.0;
};
} // namespace gnc
