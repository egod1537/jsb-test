#include "application/sim/gnc/Autopilot.hpp"

#include "application/sim/Aircraft.hpp"
#include "application/sim/Tick.hpp"
#include "application/sim/control/ControlInput.hpp"
#include "application/sim/gnc/TrimSolver.hpp"

namespace gnc {
Autopilot::Autopilot(control::IFlightControlSource &passthroughSource)
    : passthroughSource_(passthroughSource) {
  AddController<RollHoldController>();
  AddController<PitchHoldController>();
  AddController<AirspeedHoldController>();
  AddController<CourseHoldController>();
  AddController<AltitudeHoldController>();
}

void Autopilot::OnReset() { ResetControllers(); }

control::ControlInput Autopilot::OnTick(const sim::Aircraft &aircraft,
    const sim::Tick &tick) {
  control::ControlInput input = passthroughSource_.OnTick(aircraft, tick);

  if (auto *rollHold = GetController<RollHoldController>()) {
    if (const auto aileron = rollHold->OnTick(aircraft, tick)) {
      input.aileron = *aileron;
    }
  }

  if (auto *pitchHold = GetController<PitchHoldController>()) {
    if (const auto elevator = pitchHold->OnTick(aircraft, tick)) {
      input.elevator = *elevator;
    }
  }

  if (auto *airspeedHold = GetController<AirspeedHoldController>()) {
    if (const auto throttle = airspeedHold->OnTick(aircraft, tick)) {
      input.throttle = *throttle;
    }
  }

  control::ClampControlInput(input);
  return input;
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

  aircraft.GetControls().SetInput({
      .elevator = trimResult_->elevator,
      .aileron = trimResult_->aileron,
      .rudder = trimResult_->rudder,
      .throttle = trimResult_->throttle,
  });
  aircraft.GetControls().SetPitchTrim(trimResult_->pitchTrim);
  return true;
}

void Autopilot::ClearTrimResult() {
  trimResult_.reset();
  ResetControllers();
  TrimResult emptyResult{};
  SyncControllerTrimReferences(emptyResult);
}

bool Autopilot::StoreSolvedTrimResult(const TrimResult &result) {
  if (!result.success) {
    return false;
  }

  trimResult_ = result;
  ResetControllers();
  SyncControllerTrimReferences(result);
  return true;
}

void Autopilot::ResetControllers() {
  for (const auto &controller : controllers_) {
    controller->Reset();
  }
}

void Autopilot::SyncControllerTrimReferences(const TrimResult &result) {
  if (auto *rollHold = GetController<RollHoldController>()) {
    rollHold->SetTrimAileron(result.aileron);
  }
  if (auto *pitchHold = GetController<PitchHoldController>()) {
    pitchHold->SetTrimElevator(result.elevator);
  }
  if (auto *airspeedHold = GetController<AirspeedHoldController>()) {
    airspeedHold->SetTrimThrottle(result.throttle);
  }
  if (auto *courseHold = GetController<CourseHoldController>()) {
    courseHold->SetTrimAileron(result.aileron);
  }
  if (auto *altitudeHold = GetController<AltitudeHoldController>()) {
    altitudeHold->SetTrimElevator(result.elevator);
  }
}

bool Autopilot::HasTrimResult() const { return trimResult_.has_value(); }

const TrimResult *Autopilot::GetTrimResult() const {
  return trimResult_ ? &*trimResult_ : nullptr;
}

bool Autopilot::IsRollHoldEnabled() const {
  const auto *rollHold = GetController<RollHoldController>();
  return rollHold != nullptr && rollHold->IsEnabled();
}

void Autopilot::SetRollHoldEnabled(bool enabled) {
  if (auto *rollHold = GetController<RollHoldController>()) {
    rollHold->SetEnabled(enabled);
  }
}

bool Autopilot::IsPitchHoldEnabled() const {
  const auto *pitchHold = GetController<PitchHoldController>();
  return pitchHold != nullptr && pitchHold->IsEnabled();
}

void Autopilot::SetPitchHoldEnabled(bool enabled) {
  if (auto *pitchHold = GetController<PitchHoldController>()) {
    pitchHold->SetEnabled(enabled);
  }
}

void Autopilot::SetRollHoldSettings(const RollHoldSettings &settings) {
  if (auto *rollHold = GetController<RollHoldController>()) {
    rollHold->SetSettings(settings);
  }
}

const RollHoldSettings &Autopilot::GetRollHoldSettings() const {
  static const RollHoldSettings defaultSettings{};
  const auto *rollHold = GetController<RollHoldController>();
  return rollHold != nullptr ? rollHold->GetSettings() : defaultSettings;
}

void Autopilot::SetPitchHoldSettings(const PitchHoldSettings &settings) {
  if (auto *pitchHold = GetController<PitchHoldController>()) {
    pitchHold->SetSettings(settings);
  }
}

const PitchHoldSettings &Autopilot::GetPitchHoldSettings() const {
  static const PitchHoldSettings defaultSettings{};
  const auto *pitchHold = GetController<PitchHoldController>();
  return pitchHold != nullptr ? pitchHold->GetSettings() : defaultSettings;
}
} // namespace gnc
