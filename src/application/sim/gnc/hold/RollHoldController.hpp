#pragma once

#include "application/sim/gnc/Controller.hpp"
#include "application/sim/gnc/hold/RollHoldSettings.hpp"

#include <optional>

namespace sim {
class Aircraft;
struct Tick;
} // namespace sim

namespace gnc {
struct ControlContext;

class RollHoldController final : public Controller {
public:
  void Reset() override;

  bool IsEnabled() const;
  void SetEnabled(bool enabled);

  const RollHoldSettings &GetSettings() const;
  void SetSettings(const RollHoldSettings &settings);

  double GetTrimAileron() const;
  void SetTrimAileron(double trimAileron);

  // Standalone Roll Hold
  std::optional<double> OnTick(const sim::Aircraft &aircraft,
      const sim::Tick &tick, const ControlContext &context);

  // Cascaded outer-loop command
  std::optional<double> OnTick(const sim::Aircraft &aircraft,
      const sim::Tick &tick, const ControlContext &context,
      double commandedRollRad);

private:
  std::optional<double> ComputeAileronCommand(const sim::Aircraft &aircraft,
      const ControlContext &context, double targetRollRad) const;

  bool enabled_ = false;
  RollHoldSettings settings_;
  double trimAileron_ = 0.0;
};
} // namespace gnc
