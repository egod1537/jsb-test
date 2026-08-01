#pragma once

#include "application/sim/gnc/Controller.hpp"

#include <optional>

namespace sim {
class Aircraft;
struct Tick;
} // namespace sim

namespace gnc {
struct PitchHoldSettings {
  double targetPitchRad = 0.0;
  double proportionalGain = 0.5;
  double derivativeGain = 2.0;
};

class PitchHoldController final : public Controller {
public:
  void Reset() override;

  bool IsEnabled() const;
  void SetEnabled(bool enabled);

  const PitchHoldSettings &GetSettings() const;
  void SetSettings(const PitchHoldSettings &settings);

  double GetTrimElevator() const;
  void SetTrimElevator(double trimElevator);

  std::optional<double> OnTick(const sim::Aircraft &aircraft,
      const sim::Tick &tick);

private:
  bool enabled_ = false;
  PitchHoldSettings settings_;
  double trimElevator_ = 0.0;
};
} // namespace gnc
