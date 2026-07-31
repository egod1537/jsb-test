#include "application/sim/gnc/Autopilot.hpp"

#include "application/sim/Aircraft.hpp"
#include "application/sim/Context.hpp"
#include "application/sim/Tick.hpp"
#include "application/sim/gnc/TrimSolver.hpp"

namespace gnc {
const char *Autopilot::GetName() const { return "Autopilot"; }

void Autopilot::Reset() { ResetHoldControllers(); }

bool Autopilot::Initialize(sim::Context &context) {
  return rollHold_.Initialize(context) && pitchHold_.Initialize(context)
         && airspeedHold_.Initialize(context) && courseHold_.Initialize(context)
         && altitudeHold_.Initialize(context);
}

bool Autopilot::Reset(sim::Context &context) {
  return rollHold_.Reset(context) && pitchHold_.Reset(context)
         && airspeedHold_.Reset(context) && courseHold_.Reset(context)
         && altitudeHold_.Reset(context);
}

bool Autopilot::PreStep(sim::Context &context, const sim::Tick &tick) {
  return true;
}

bool Autopilot::ComputeTrim(sim::Aircraft &aircraft,
    const TrimRequest &request) {
  return StoreSolvedTrimResult(TrimSolver::Solve(aircraft, request));
}

bool Autopilot::ComputeCurrentStateTrim(sim::Aircraft &aircraft,
    TrimMode mode) {
  return StoreSolvedTrimResult(TrimSolver::SolveCurrentState(aircraft, mode));
}

bool Autopilot::ApplyStoredTrim(sim::Aircraft &aircraft) {
  if (!trimResult_ || !trimResult_->success) {
    return false;
  }

  aircraft.SetAircraftControlInput({
      .elevator = trimResult_->elevator,
      .aileron = trimResult_->aileron,
      .rudder = trimResult_->rudder,
      .throttle = trimResult_->throttle,
  });
  aircraft.GetFlightControls().SetPitchTrim(trimResult_->pitchTrim);
  return true;
}

void Autopilot::ClearTrimResult() {
  trimResult_.reset();
  ResetHoldControllers();
  TrimResult emptyResult{};
  UpdateControllerTrimReferences(emptyResult);
}

bool Autopilot::StoreSolvedTrimResult(const TrimResult &result) {
  if (!result.success) {
    return false;
  }

  trimResult_ = result;
  ResetHoldControllers();
  UpdateControllerTrimReferences(result);
  return true;
}

void Autopilot::ResetHoldControllers() {
  rollHold_.Reset();
  pitchHold_.Reset();
  airspeedHold_.Reset();
  courseHold_.Reset();
  altitudeHold_.Reset();
}

void Autopilot::UpdateControllerTrimReferences(const TrimResult &result) {
  rollHold_.SetTrimAileron(result.aileron);
  pitchHold_.SetTrimElevator(result.elevator);
  airspeedHold_.SetTrimThrottle(result.throttle);
  courseHold_.SetTrimAileron(result.aileron);
  altitudeHold_.SetTrimElevator(result.elevator);
}

bool Autopilot::HasTrimResult() const { return trimResult_.has_value(); }

const TrimResult *Autopilot::GetTrimResult() const {
  return trimResult_ ? &*trimResult_ : nullptr;
}

RollHoldController &Autopilot::GetRollHoldController() {
  return rollHold_;
}

const RollHoldController &Autopilot::GetRollHoldController() const {
  return rollHold_;
}

PitchHoldController &Autopilot::GetPitchHoldController() {
  return pitchHold_;
}

const PitchHoldController &Autopilot::GetPitchHoldController() const {
  return pitchHold_;
}

AirspeedHoldController &Autopilot::GetAirspeedHoldController() {
  return airspeedHold_;
}

const AirspeedHoldController &Autopilot::GetAirspeedHoldController() const {
  return airspeedHold_;
}

CourseHoldController &Autopilot::GetCourseHoldController() {
  return courseHold_;
}

const CourseHoldController &Autopilot::GetCourseHoldController() const {
  return courseHold_;
}

AltitudeHoldController &Autopilot::GetAltitudeHoldController() {
  return altitudeHold_;
}

const AltitudeHoldController &Autopilot::GetAltitudeHoldController() const {
  return altitudeHold_;
}

bool Autopilot::IsRollHoldEnabled() const { return rollHold_.IsEnabled(); }

void Autopilot::SetRollHoldEnabled(bool enabled) {
  rollHold_.SetEnabled(enabled);
}

bool Autopilot::IsPitchHoldEnabled() const { return pitchHold_.IsEnabled(); }

void Autopilot::SetPitchHoldEnabled(bool enabled) {
  pitchHold_.SetEnabled(enabled);
}

void Autopilot::SetRollHoldSettings(const RollHoldSettings &settings) {
  rollHold_.SetSettings(settings);
}

const RollHoldSettings &Autopilot::GetRollHoldSettings() const {
  return rollHold_.GetSettings();
}

void Autopilot::SetPitchHoldSettings(const PitchHoldSettings &settings) {
  pitchHold_.SetSettings(settings);
}

const PitchHoldSettings &Autopilot::GetPitchHoldSettings() const {
  return pitchHold_.GetSettings();
}
} // namespace gnc
