#pragma once

#include "application/sim/System.hpp"
#include "application/sim/gnc/hold/AirspeedHoldController.hpp"
#include "application/sim/gnc/hold/AltitudeHoldController.hpp"
#include "application/sim/gnc/hold/CourseHoldController.hpp"
#include "application/sim/gnc/hold/PitchHoldController.hpp"
#include "application/sim/gnc/hold/RollHoldController.hpp"
#include "application/sim/gnc/TrimTypes.hpp"

#include <optional>

namespace gnc {
class Autopilot final : public sim::System {
public:
  const char *GetName() const;
  void Reset();

  bool Initialize(sim::Context &context) override;
  bool Reset(sim::Context &context) override;
  bool PreStep(sim::Context &context, const sim::Tick &tick) override;

  bool ComputeTrim(sim::Aircraft &aircraft, const TrimRequest &request);
  bool ComputeCurrentStateTrim(sim::Aircraft &aircraft,
      TrimMode mode = TrimMode::Longitudinal);
  bool ApplyStoredTrim(sim::Aircraft &aircraft);

  void ClearTrimResult();
  bool HasTrimResult() const;
  const TrimResult *GetTrimResult() const;

  RollHoldController &GetRollHoldController();
  const RollHoldController &GetRollHoldController() const;
  PitchHoldController &GetPitchHoldController();
  const PitchHoldController &GetPitchHoldController() const;
  AirspeedHoldController &GetAirspeedHoldController();
  const AirspeedHoldController &GetAirspeedHoldController() const;
  CourseHoldController &GetCourseHoldController();
  const CourseHoldController &GetCourseHoldController() const;
  AltitudeHoldController &GetAltitudeHoldController();
  const AltitudeHoldController &GetAltitudeHoldController() const;

  bool IsRollHoldEnabled() const;
  void SetRollHoldEnabled(bool enabled);
  bool IsPitchHoldEnabled() const;
  void SetPitchHoldEnabled(bool enabled);

  void SetRollHoldSettings(const RollHoldSettings &settings);
  const RollHoldSettings &GetRollHoldSettings() const;
  void SetPitchHoldSettings(const PitchHoldSettings &settings);
  const PitchHoldSettings &GetPitchHoldSettings() const;

private:
  bool StoreSolvedTrimResult(const TrimResult &result);
  void ResetHoldControllers();
  void UpdateControllerTrimReferences(const TrimResult &result);

  std::optional<TrimResult> trimResult_;
  RollHoldController rollHold_;
  PitchHoldController pitchHold_;
  AirspeedHoldController airspeedHold_;
  CourseHoldController courseHold_;
  AltitudeHoldController altitudeHold_;
};
} // namespace gnc
