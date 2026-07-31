#include "application/sim/gnc/TrimSolver.hpp"

#include "application/sim/Aircraft.hpp"
#include "application/sim/InitialCondition.hpp"

#include <FGFDMExec.h>
#include <exception>
#include <initialization/FGInitialCondition.h>
#include <initialization/FGTrim.h>
#include <iostream>

namespace {
constexpr const char *SetAllEnginesRunning = "propulsion/set-running";

using gnc::TrimMode;
using gnc::TrimRequest;
using gnc::TrimResult;

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

void ApplyTrimRequestInitialConditions(sim::Aircraft &aircraft,
    const TrimRequest &req) {
  auto initialCondition = aircraft.GetFDMExec().GetIC();
  if (req.mode == TrimMode::Ground) {
    return;
  }

  initialCondition->SetVcalibratedKtsIC(req.airspeedKts);
  initialCondition->SetAltitudeASLFtIC(req.altitudeFt);
  initialCondition->SetFlightPathAngleDegIC(req.flightPathAngleDeg);
}

void PreparePropulsionForTrim(sim::Aircraft &aircraft,
    TrimMode mode) {
  if (mode == TrimMode::Ground) {
    return;
  }

  aircraft.GetProperties().Set(SetAllEnginesRunning, -1.0);
}

TrimResult BuildTrimResult(const sim::Aircraft &aircraft) {
  const auto &properties = aircraft.GetProperties();
  const auto &flightControls = aircraft.GetFlightControls();

  TrimResult result{};
  result.success = true;
  result.alphaDeg = properties.Alpha().Deg();
  result.betaDeg = properties.Beta().Deg();
  result.rollDeg = properties.Roll().Deg();
  result.pitchDeg = properties.Pitch().Deg();

  result.throttle = flightControls.GetThrottle();
  result.elevator = flightControls.GetElevator();
  result.pitchTrim = flightControls.GetPitchTrim();
  result.aileron = flightControls.GetAileron();
  result.rudder = flightControls.GetRudder();

  result.uDot = properties.U().DotMps2();
  result.vDot = properties.V().DotMps2();
  result.wDot = properties.W().DotMps2();
  result.pDot = properties.P().DotDegPerSec2();
  result.qDot = properties.Q().DotDegPerSec2();
  result.rDot = properties.R().DotDegPerSec2();

  return result;
}

int ToJSBTrimMode(TrimMode mode) {
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

TrimResult ExecuteTrim(sim::Aircraft &aircraft, TrimMode mode,
    bool runInitialCondition) {
  auto &properties = aircraft.GetProperties();
  auto &fdm = aircraft.GetFDMExec();

  std::cout << "[Trim] begin mode=" << TrimModeName(mode)
            << " simTime=" << properties.SimTime().Sec() << '\n';

  try {
    if (runInitialCondition) {
      if (!fdm.RunIC()) {
        std::cout << "[Trim] RunIC failed simTime="
                  << properties.SimTime().Sec() << '\n';
        return {
            .success = false,
            .message = "Failed to apply initial conditions.",
        };
      }

      std::cout << "[Trim] RunIC simTime=" << properties.SimTime().Sec()
                << '\n';
    }

    PreparePropulsionForTrim(aircraft, mode);

    const int jsbMode = ToJSBTrimMode(mode);
    fdm.DoTrim(jsbMode);

    std::cout << "[Trim] DoTrim simTime=" << properties.SimTime().Sec()
              << '\n';

    TrimResult result = BuildTrimResult(aircraft);

    std::cout << "[Trim] end success=true simTime="
              << properties.SimTime().Sec() << '\n';

    return result;
  } catch (const std::exception &e) {
    std::cout << "[Trim] end success=false simTime="
              << properties.SimTime().Sec() << " message=" << e.what()
              << '\n';

    TrimResult result{};
    result.success = false;
    result.message = e.what();

    return result;
  }
}
} // namespace

namespace gnc::TrimSolver {
TrimResult Solve(sim::Aircraft &aircraft, const TrimRequest &req) {
  ApplyTrimRequestInitialConditions(aircraft, req);
  return ExecuteTrim(aircraft, req.mode, true);
}

TrimResult SolveCurrentState(sim::Aircraft &aircraft, TrimMode mode) {
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
} // namespace gnc::TrimSolver
