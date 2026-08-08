#pragma once

#include "application/sim/gnc/Controller.hpp"

#include <optional>

namespace sim {
class Aircraft;
struct Tick;
} // namespace sim

namespace gnc {
struct YawDamperSettings {
  double yawRateGain = 0.0;
};

class YawDamperController final : public Controller {
public:
  void Reset() override;

  bool IsEnabled() const;
  void SetEnabled(bool enabled);

  const YawDamperSettings &GetSettings() const;
  void SetSettings(const YawDamperSettings &settings);

  double GetTrimRudder() const;
  void SetTrimRudder(double trimRudder);

  std::optional<double> OnTick(const sim::Aircraft &aircraft,
      const sim::Tick &tick);

private:
  bool enabled_ = false;
  YawDamperSettings settings_;
  double trimRudder_ = 0.0;
};
} // namespace gnc
