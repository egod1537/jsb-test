#pragma once

#include "application/sim/Aircraft.hpp"
#include "application/sim/Tick.hpp"
#include "application/sim/control/IFlightControlSource.hpp"
#include "application/sim/gnc/Controller.hpp"
#include "application/sim/gnc/hold/AirspeedHoldController.hpp"
#include "application/sim/gnc/hold/CourseHoldController.hpp"
#include "application/sim/gnc/hold/PitchHoldController.hpp"
#include "application/sim/gnc/hold/PitchDynamics.hpp"
#include "application/sim/gnc/hold/RollDynamics.hpp"
#include "application/sim/gnc/hold/RollHoldController.hpp"
#include "application/sim/gnc/TrimTypes.hpp"
#include "application/sim/linearizer/LinearizationResult.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace sim {
class AsyncAircraftLinearizer;
}

namespace gnc {
class Autopilot final : public control::IFlightControlSource {
public:
  explicit Autopilot(control::IFlightControlSource &passthroughSource);
  ~Autopilot() override;

  // Lifecycle and control output
  void OnReset();
  control::ControlInput OnTick(sim::Aircraft &aircraft,
      const sim::Tick &tick) override;

  // Periodic aircraft dynamics
  void UpdateLinearization(sim::Aircraft &aircraft, const sim::Tick &tick);

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
  bool IsCourseHoldEnabled() const;
  void SetCourseHoldEnabled(bool enabled);

  // Hold settings
  void SetRollHoldSettings(const RollHoldSettings &settings);
  const RollHoldSettings &GetRollHoldSettings() const;
  void SetPitchHoldSettings(const PitchHoldSettings &settings);
  const PitchHoldSettings &GetPitchHoldSettings() const;
  void SetCourseHoldSettings(const CourseHoldSettings &settings);
  const CourseHoldSettings &GetCourseHoldSettings() const;

  // Linearization
  bool IsLinearizationInProgress() const;
  const LinearizationResult *GetLinearizationResult() const;
  std::string_view GetLinearizationErrorMessage() const;
  std::optional<RollDynamics> GetRollDynamics() const;
  std::optional<PitchDynamics> GetPitchDynamics() const;

private:
  // Trim management
  bool StoreSolvedTrimResult(const TrimResult &result);
  void ResetControllers();
  void SyncControllerTrimReferences(const TrimResult &result);

  // Aircraft dynamics
  void PollLinearization();
  bool SubmitLinearization(sim::Aircraft &aircraft, double simulationTimeSec);
  void InvalidateLinearization();

  // Input dependency
  control::IFlightControlSource &passthroughSource_;

  // Cached trim
  std::optional<TrimResult> trimResult_;

  // Controller ownership
  std::vector<std::unique_ptr<Controller>> controllers_;

  // Aircraft dynamics
  std::unique_ptr<sim::AsyncAircraftLinearizer> asyncLinearizer_;
  std::optional<LinearizationResult> linearization_;
  std::string linearizationErrorMessage_;
  std::optional<double> lastLinearizationCycleSimTimeSec_;
  std::uint64_t linearizationGeneration_ = 0;
};

} // namespace gnc
#include "application/sim/gnc/Autopilot.inl"
