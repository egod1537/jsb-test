#include "gnc/TrimSolver.hpp"

#include "simulation/Aircraft.hpp"
#include "simulation/InitialCondition.hpp"

#include <FGFDMExec.h>
#include <exception>
#include <initialization/FGInitialCondition.h>
#include <initialization/FGTrim.h>
#include <iostream>

namespace {
constexpr const char *SetAllEnginesRunning = "propulsion/set-running";

const char *TrimModeName(gnc::TrimMode mode) {
  switch (mode) {
  case gnc::TrimMode::Longitudinal:
    return "Longitudinal";
  case gnc::TrimMode::Full:
    return "Full";
  case gnc::TrimMode::Ground:
    return "Ground";
  }

  return "Unknown";
}
} // namespace

namespace gnc {
TrimResult TrimSolver::Trim(sim::Aircraft &aircraft, const TrimRequest &req) {
  ApplyTrimRequestInitialConditions(aircraft, req);
  return ExecuteTrim(aircraft, req.mode, true);
}

TrimResult TrimSolver::TrimCurrentState(sim::Aircraft &aircraft,
    TrimMode mode) {
  const sim::InitialCondition currentCondition =
      aircraft.CaptureCurrentCondition();
  if (!aircraft.ApplyInitialCondition(currentCondition)) {
    return {
        .success = false,
        .message = "Failed to apply current state as initial condition.",
    };
  }

  return ExecuteTrim(aircraft, mode, false);
}

void TrimSolver::ApplyTrimRequestInitialConditions(sim::Aircraft &aircraft,
    const TrimRequest &req) {
  auto initialCondition = aircraft.GetFDMExec().GetIC();
  if (req.mode == TrimMode::Ground) {
    return;
  }

  initialCondition->SetVcalibratedKtsIC(req.airspeedKts);
  initialCondition->SetAltitudeASLFtIC(req.altitudeFt);
  initialCondition->SetFlightPathAngleDegIC(req.flightPathAngleDeg);
}

void TrimSolver::PreparePropulsionForTrim(sim::Aircraft &aircraft,
    TrimMode mode) {
  if (mode == TrimMode::Ground) {
    return;
  }

  aircraft.GetProperties().Set(SetAllEnginesRunning, -1.0);
}

TrimResult TrimSolver::ExecuteTrim(sim::Aircraft &aircraft, TrimMode mode,
    bool runInitialCondition) {
  const std::uint64_t trimId = ++executionCount_;
  auto &properties = aircraft.GetProperties();
  auto &fdm = aircraft.GetFDMExec();

  std::cout << "[Trim] begin id=" << trimId << " mode=" << TrimModeName(mode)
            << " simTime=" << properties.GetSimTimeSec() << '\n';

  try {
    if (runInitialCondition) {
      if (!fdm.RunIC()) {
        std::cout << "[Trim] RunIC failed id=" << trimId
                  << " simTime=" << properties.GetSimTimeSec() << '\n';
        return {
            .success = false,
            .message = "Failed to apply initial conditions.",
        };
      }

      std::cout << "[Trim] RunIC id=" << trimId
                << " simTime=" << properties.GetSimTimeSec() << '\n';
    }

    PreparePropulsionForTrim(aircraft, mode);

    const int jsbMode = ToJSBTrimMode(mode);
    fdm.DoTrim(jsbMode);

    std::cout << "[Trim] DoTrim id=" << trimId
              << " simTime=" << properties.GetSimTimeSec() << '\n';

    TrimResult result = BuildTrimResult(aircraft);
    ApplyTrimResultToAircraft(aircraft, result);

    std::cout << "[Trim] end id=" << trimId
              << " success=true simTime=" << properties.GetSimTimeSec() << '\n';

    return result;
  } catch (const std::exception &e) {
    std::cout << "[Trim] end id=" << trimId
              << " success=false simTime=" << properties.GetSimTimeSec()
              << " message=" << e.what() << '\n';

    TrimResult result{};
    result.success = false;
    result.message = e.what();

    return result;
  }
}

TrimResult TrimSolver::BuildTrimResult(const sim::Aircraft &aircraft) const {
  const auto &properties = aircraft.GetProperties();
  const auto &flightControls = aircraft.GetFlightControls();

  TrimResult result{};
  result.success = true;
  result.alphaDeg = properties.GetAlphaDeg();
  result.betaDeg = properties.GetBetaDeg();
  result.rollDeg = properties.GetRollDeg();
  result.pitchDeg = properties.GetPitchDeg();

  result.throttle = flightControls.GetThrottle();
  result.elevator = flightControls.GetElevator();
  result.pitchTrim = flightControls.GetPitchTrim();
  result.aileron = flightControls.GetAileron();
  result.rudder = flightControls.GetRudder();

  result.uDot = properties.GetUDotMps2();
  result.vDot = properties.GetVDotMps2();
  result.wDot = properties.GetWDotMps2();
  result.pDot = properties.GetPdotDegPerSec2();
  result.qDot = properties.GetQdotDegPerSec2();
  result.rDot = properties.GetRdotDegPerSec2();

  return result;
}

void TrimSolver::ApplyTrimResultToAircraft(sim::Aircraft &aircraft,
    const TrimResult &result) {
  aircraft.SetAircraftControlInput({
      .elevator = result.elevator,
      .aileron = result.aileron,
      .rudder = result.rudder,
      .throttle = result.throttle,
  });
  aircraft.GetFlightControls().SetPitchTrim(result.pitchTrim);
}

int TrimSolver::ToJSBTrimMode(TrimMode mode) {
  switch (mode) {
  case TrimMode::Longitudinal:
    return JSBSim::tLongitudinal;
  case TrimMode::Full:
    return JSBSim::tFull;
  case TrimMode::Ground:
    return JSBSim::tGround;
  }

  return JSBSim::tNone;
}
} // namespace gnc
