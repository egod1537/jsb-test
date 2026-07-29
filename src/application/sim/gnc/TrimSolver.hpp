#pragma once

#include "application/sim/gnc/TrimTypes.hpp"

#include <cstdint>

namespace sim {
class Aircraft;
}

namespace gnc {
class TrimSolver {
public:
  TrimResult Trim(sim::Aircraft &aircraft, const TrimRequest &request);
  TrimResult TrimCurrentState(sim::Aircraft &aircraft,
      TrimMode mode = TrimMode::Longitudinal);

private:
  void ApplyTrimRequestInitialConditions(sim::Aircraft &aircraft,
      const TrimRequest &request);
  void PreparePropulsionForTrim(sim::Aircraft &aircraft, TrimMode mode);
  TrimResult ExecuteTrim(sim::Aircraft &aircraft, TrimMode mode,
      bool runInitialCondition);
  TrimResult BuildTrimResult(const sim::Aircraft &aircraft) const;
  void ApplyTrimResultToAircraft(sim::Aircraft &aircraft,
      const TrimResult &result);

  static int ToJSBTrimMode(TrimMode mode);

  std::uint64_t executionCount_ = 0;
};
} // namespace gnc
