#pragma once

#include "application/sim/control/IFlightControlSource.hpp"
#include "application/sim/gnc/Controller.hpp"
#include "application/sim/gnc/hold/AirspeedHoldController.hpp"
#include "application/sim/gnc/hold/AltitudeHoldController.hpp"
#include "application/sim/gnc/hold/CourseHoldController.hpp"
#include "application/sim/gnc/hold/PitchHoldController.hpp"
#include "application/sim/gnc/hold/RollHoldController.hpp"
#include "application/sim/gnc/TrimTypes.hpp"

#include <memory>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

namespace gnc {
class Autopilot final : public control::IFlightControlSource {
public:
  explicit Autopilot(control::IFlightControlSource &passthroughSource);

  // Lifecycle and control output
  void OnReset();
  control::ControlInput OnTick(const sim::Aircraft &aircraft,
      const sim::Tick &tick) override;

  // Controller registry
  template <typename T, typename... Args> T *AddController(Args &&...args);
  template <typename T> T *GetController();
  template <typename T> const T *GetController() const;
  template <typename T> bool RemoveController();

  // Trim
  bool ComputeTrim(sim::Aircraft &aircraft, const TrimRequest &request);
  bool ComputeCurrentStateTrim(sim::Aircraft &aircraft,
      TrimMode mode = TrimMode::Longitudinal);
  bool ApplyStoredTrim(sim::Aircraft &aircraft);

  void ClearTrimResult();
  bool HasTrimResult() const;
  const TrimResult *GetTrimResult() const;

  // Hold state
  bool IsRollHoldEnabled() const;
  void SetRollHoldEnabled(bool enabled);
  bool IsPitchHoldEnabled() const;
  void SetPitchHoldEnabled(bool enabled);

  // Hold settings
  void SetRollHoldSettings(const RollHoldSettings &settings);
  const RollHoldSettings &GetRollHoldSettings() const;
  void SetPitchHoldSettings(const PitchHoldSettings &settings);
  const PitchHoldSettings &GetPitchHoldSettings() const;

private:
  // Trim management
  bool StoreSolvedTrimResult(const TrimResult &result);
  void ResetControllers();
  void SyncControllerTrimReferences(const TrimResult &result);

  // Input dependency
  control::IFlightControlSource &passthroughSource_;

  // Cached trim
  std::optional<TrimResult> trimResult_;

  // Controller ownership
  std::vector<std::unique_ptr<Controller>> controllers_;
};
} // namespace gnc

#include "application/sim/gnc/Autopilot.inl"
