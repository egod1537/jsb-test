#include "application/sim/gnc/Autopilot.hpp"

#include "application/sim/Aircraft.hpp"
#include "application/sim/FDMState.hpp"
#include "application/sim/Tick.hpp"
#include "application/sim/control/ControlInput.hpp"
#include "application/sim/gnc/ControlContext.hpp"
#include "application/sim/gnc/TrimSolver.hpp"
#include "application/sim/gnc/hold/CourseHoldController.hpp"
#include "application/sim/gnc/hold/AltitudeHoldController.hpp"
#include "application/sim/gnc/hold/RollDynamics.hpp"
#include "application/sim/gnc/hold/PitchDynamics.hpp"
#include "application/sim/linearizer/AsyncAircraftLinearizer.hpp"
#include <cstdint>
#include <iostream>
#include <optional>
#include <utility>

namespace {
constexpr double LinearizationRefreshIntervalSec = 5.0;
}

namespace gnc {
Autopilot::Autopilot(control::IFlightControlSource &passthroughSource)
    : passthroughSource_(passthroughSource),
      asyncLinearizer_(std::make_unique<sim::AsyncAircraftLinearizer>()) {
  AddController<RollHoldController>();
  AddController<PitchHoldController>();
  AddController<AirspeedHoldController>();
  AddController<CourseHoldController>();
  AddController<AltitudeHoldController>();
}

Autopilot::~Autopilot() = default;

void Autopilot::OnReset() {
  ResetControllers();
  InvalidateLinearization();
}

control::ControlInput Autopilot::OnTick(sim::Aircraft &aircraft,
    const sim::Tick &tick) {
  control::ControlInput input = passthroughSource_.OnTick(aircraft, tick);
  UpdateLinearization(aircraft, tick);

  const ControlContext context{
      .rollDynamics = GetRollDynamics(),
      .pitchDynamics = GetPitchDynamics(),
  };

  if (auto *rollHold = GetController<RollHoldController>()) {
    if (context.rollDynamics) {
      if (const auto aileron = rollHold->OnTick(aircraft, tick, context)) {
        input.aileron = *aileron;
      }
    }
  }

  if (auto *pitchHold = GetController<PitchHoldController>()) {
    if (context.pitchDynamics) {
      if (const auto elevator = pitchHold->OnTick(aircraft, tick, context)) {
        input.elevator = *elevator;
      }
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

std::optional<RollDynamics> Autopilot::GetRollDynamics() const {
  if (!linearization_) {
    return std::nullopt;
  }

  const auto &A = linearization_->A;
  const auto &B = linearization_->B;

  const auto p = linearization_->FindStateIndex("P");
  const auto da = linearization_->FindInputIndex("DaCmd");

  if (!p || !da) {
    return std::nullopt;
  }

  return RollDynamics{
      .aPhi1 = -A(*p, *p),
      .aPhi2 = B(*p, *da),
  };
}

std::optional<PitchDynamics> Autopilot::GetPitchDynamics() const {
  if (!linearization_) {
    return std::nullopt;
  }

  const auto &A = linearization_->A;
  const auto &B = linearization_->B;

  const auto alpha = linearization_->FindStateIndex("Alpha");
  const auto q = linearization_->FindStateIndex("Q");
  const auto de = linearization_->FindInputIndex("DeCmd");

  const auto p = linearization_->FindStateIndex("P");
  const auto r = linearization_->FindStateIndex("R");

  if (!alpha || !q || !de) {
    return std::nullopt;
  }

  return PitchDynamics{
      .aTheta1 = -A(*q, *q),
      .aTheta2 = -A(*q, *alpha),
      .aTheta3 = B(*q, *de),
  };
}

void Autopilot::UpdateLinearization(sim::Aircraft &aircraft,
    const sim::Tick &tick) {
  if (auto completion = asyncLinearizer_->TakeCompletion()) {
    if (completion->generation == linearizationGeneration_) {
      lastLinearizationRequestSimTimeSec_ = tick.simTimeSec;
      if (completion->linearization) {
        linearization_ = std::move(completion->linearization);
      } else if (!completion->errorMessage.empty()) {
        std::cerr << "[Autopilot] " << completion->errorMessage << '\n';
      }
    }
  }

  const auto *rollHold = GetController<RollHoldController>();
  const auto *pitchHold = GetController<PitchHoldController>();
  const bool dynamicsRequired =
      (rollHold != nullptr && rollHold->IsEnabled())
      || (pitchHold != nullptr && pitchHold->IsEnabled());
  if (!dynamicsRequired || asyncLinearizer_->IsBusy()) {
    return;
  }

  const bool simulationTimeReset =
      lastLinearizationRequestSimTimeSec_
      && tick.simTimeSec < *lastLinearizationRequestSimTimeSec_;
  const bool refreshDue =
      !lastLinearizationRequestSimTimeSec_ || simulationTimeReset
      || tick.simTimeSec - *lastLinearizationRequestSimTimeSec_
             >= LinearizationRefreshIntervalSec;
  if (!refreshDue) {
    return;
  }

  sim::FDMState sourceState = aircraft.ExtractFDMState(sim::FDMStateFlags::All);
  if (asyncLinearizer_->Submit(linearizationGeneration_,
          aircraft.GetConfig(),
          aircraft.GetCurrentCondition(),
          std::move(sourceState))) {
    lastLinearizationRequestSimTimeSec_ = tick.simTimeSec;
  }
}

void Autopilot::InvalidateLinearization() {
  linearization_.reset();
  lastLinearizationRequestSimTimeSec_.reset();
  ++linearizationGeneration_;
}

bool Autopilot::ComputeTrim(sim::Aircraft &aircraft,
    const TrimRequest &request) {
  const bool succeeded =
      StoreSolvedTrimResult(TrimSolver::Solve(aircraft, request));
  if (succeeded) {
    InvalidateLinearization();
  }
  return succeeded;
}

bool Autopilot::ComputeCurrentStateTrim(sim::Aircraft &aircraft,
    TrimMode mode) {
  const bool succeeded =
      StoreSolvedTrimResult(TrimSolver::SolveCurrentState(aircraft, mode));
  if (succeeded) {
    InvalidateLinearization();
  }
  return succeeded;
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
  static const RollHoldSettings DefaultSettings{};
  const auto *rollHold = GetController<RollHoldController>();
  return rollHold != nullptr ? rollHold->GetSettings() : DefaultSettings;
}

void Autopilot::SetPitchHoldSettings(const PitchHoldSettings &settings) {
  if (auto *pitchHold = GetController<PitchHoldController>()) {
    pitchHold->SetSettings(settings);
  }
}

const PitchHoldSettings &Autopilot::GetPitchHoldSettings() const {
  static const PitchHoldSettings DefaultSettings{};
  const auto *pitchHold = GetController<PitchHoldController>();
  return pitchHold != nullptr ? pitchHold->GetSettings() : DefaultSettings;
}
} // namespace gnc
