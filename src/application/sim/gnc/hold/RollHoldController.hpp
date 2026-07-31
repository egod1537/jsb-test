#pragma once

#include "application/sim/System.hpp"

#include <optional>

namespace sim {
class Aircraft;
}

namespace gnc {
struct RollHoldSettings {
  double targetRollRad = 0.0;
  double proportionalGain = 0.5;
  double derivativeGain = 2.0;
};

class RollHoldController final : public sim::System {
public:
  void Reset();
  bool Reset(sim::Context &context) override;

  bool IsEnabled() const;
  void SetEnabled(bool enabled);

  const RollHoldSettings &GetSettings() const;
  void SetSettings(const RollHoldSettings &settings);

  double GetTrimAileron() const;
  void SetTrimAileron(double trimAileron);

  std::optional<double> Update(const sim::Aircraft &aircraft, double dt);
  bool PreStep(sim::Context &context, const sim::Tick &tick) override;

private:
  bool enabled_ = false;
  RollHoldSettings settings_;
  double trimAileron_ = 0.0;
};
} // namespace gnc
