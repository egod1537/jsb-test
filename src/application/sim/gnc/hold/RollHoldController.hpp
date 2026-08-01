#pragma once

#include "application/sim/gnc/Controller.hpp"

#include <optional>

namespace sim {
class Aircraft;
struct Tick;
} // namespace sim

namespace gnc {
struct RollHoldSettings {
  double targetRollRad = 0.0;
  double proportionalGain = 0.5;
  double derivativeGain = 2.0;
};

class RollHoldController final : public Controller {
public:
  void Reset() override;

  bool IsEnabled() const;
  void SetEnabled(bool enabled);

  const RollHoldSettings &GetSettings() const;
  void SetSettings(const RollHoldSettings &settings);

  double GetTrimAileron() const;
  void SetTrimAileron(double trimAileron);

  std::optional<double> OnTick(const sim::Aircraft &aircraft,
      const sim::Tick &tick);

private:
  bool enabled_ = false;
  RollHoldSettings settings_;
  double trimAileron_ = 0.0;
};
} // namespace gnc
