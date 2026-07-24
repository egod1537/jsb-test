#include "simulation/FlightDynamics.hpp"
#include "gnc/TrimTypes.hpp"
#include "initialization/FGTrim.h"

#include <FGFDMExec.h>
#include <algorithm>
#include <exception>
#include <initialization/FGInitialCondition.h>
#include <iostream>
#include <simgear/misc/sg_path.hxx>

namespace {
const std::string BUILD_PATH = "build/debug";
const std::string JSBSIM_ROOT_PATH = BUILD_PATH + "/_deps/jsbsim-src";

constexpr const char *CurrentAltitudeAslFt = "position/h-sl-ft";
constexpr const char *CurrentLatitudeRad = "position/lat-gc-rad";
constexpr const char *CurrentLongitudeRad = "position/long-gc-rad";
constexpr const char *CurrentRollRad = "attitude/phi-rad";
constexpr const char *CurrentPitchRad = "attitude/theta-rad";
constexpr const char *CurrentHeadingRad = "attitude/psi-rad";
constexpr const char *CurrentUFps = "velocities/u-fps";
constexpr const char *CurrentVFps = "velocities/v-fps";
constexpr const char *CurrentWFps = "velocities/w-fps";
constexpr const char *CurrentPRadPerSec = "velocities/p-rad_sec";
constexpr const char *CurrentQRadPerSec = "velocities/q-rad_sec";
constexpr const char *CurrentRRadPerSec = "velocities/r-rad_sec";
constexpr const char *SetAllEnginesRunning = "propulsion/set-running";

double ClampUnit(double value) { return std::clamp(value, 0.0, 1.0); }

double ClampSignedUnit(double value) { return std::clamp(value, -1.0, 1.0); }
} // namespace

namespace sim {
FlightDynamics::FlightDynamics()
    : fdm_(std::make_unique<JSBSim::FGFDMExec>()), properties_(*fdm_),
      flightControls_(*fdm_) {}

FlightDynamics::~FlightDynamics() = default;

bool FlightDynamics::Initialize(const SimulationConfig &config) {
  controlInput_ = {};

  ConfigurePaths();
  if (!LoadAircraft(config)) {
    return false;
  }

  ConfigureSimulation(config);
  ConfigureInitialConditions(config);
  return InitializeState();
}

bool FlightDynamics::Update() {
  ApplyControlInput();
  return fdm_->Run();
}

JSBSim::FGFDMExec &FlightDynamics::GetFDMExec() { return *fdm_; }

const JSBSim::FGFDMExec &FlightDynamics::GetFDMExec() const { return *fdm_; }

JSBSim::FlightProperties &FlightDynamics::GetProperties() {
  return properties_;
}

const JSBSim::FlightProperties &FlightDynamics::GetProperties() const {
  return properties_;
}

JSBSim::FlightControls &FlightDynamics::GetFlightControls() {
  return flightControls_;
}

const JSBSim::FlightControls &FlightDynamics::GetFlightControls() const {
  return flightControls_;
}

control::ControlInput &FlightDynamics::GetControlInput() {
  return controlInput_;
}

const control::ControlInput &FlightDynamics::GetControlInput() const {
  return controlInput_;
}

void FlightDynamics::SetElevatorInput(double value) {
  controlInput_.elevator = ClampSignedUnit(value);
}

void FlightDynamics::SetAileronInput(double value) {
  controlInput_.aileron = ClampSignedUnit(value);
}

void FlightDynamics::SetRudderInput(double value) {
  controlInput_.rudder = ClampSignedUnit(value);
}

void FlightDynamics::SetThrottleInput(double value) {
  controlInput_.throttle = ClampUnit(value);
}

int FlightDynamics::ToJSBTrimMode(gnc::TrimMode mode) {
  switch (mode) {
  case gnc::TrimMode::Longitudinal:
    return JSBSim::tLongitudinal;
  case gnc::TrimMode::Full:
    return JSBSim::tFull;
  case gnc::TrimMode::Ground:
    return JSBSim::tGround;
  }

  return JSBSim::tNone;
}

gnc::TrimResult FlightDynamics::Trim(const gnc::TrimRequest &req) {
  ApplyTrimRequestInitialConditions(req);
  return ExecuteTrim(req.mode);
}

gnc::TrimResult FlightDynamics::TrimCurrentState(gnc::TrimMode mode) {
  ApplyCurrentStateInitialConditions();
  return ExecuteTrim(mode);
}

void FlightDynamics::ApplyTrimRequestInitialConditions(
    const gnc::TrimRequest &req) {
  auto initialCondition = fdm_->GetIC();
  if (req.mode == gnc::TrimMode::Ground) {
    return;
  }

  initialCondition->SetVcalibratedKtsIC(req.airspeedKts);
  initialCondition->SetAltitudeASLFtIC(req.altitudeFt);
  initialCondition->SetFlightPathAngleDegIC(req.flightPathAngleDeg);
}

void FlightDynamics::ApplyCurrentStateInitialConditions() {
  auto initialCondition = fdm_->GetIC();

  initialCondition->SetLatitudeRadIC(properties_.Get(CurrentLatitudeRad));
  initialCondition->SetLongitudeRadIC(properties_.Get(CurrentLongitudeRad));
  initialCondition->SetAltitudeASLFtIC(properties_.Get(CurrentAltitudeAslFt));
  initialCondition->SetPhiRadIC(properties_.Get(CurrentRollRad));
  initialCondition->SetThetaRadIC(properties_.Get(CurrentPitchRad));
  initialCondition->SetPsiRadIC(properties_.Get(CurrentHeadingRad));

  initialCondition->SetUBodyFpsIC(properties_.Get(CurrentUFps));
  initialCondition->SetVBodyFpsIC(properties_.Get(CurrentVFps));
  initialCondition->SetWBodyFpsIC(properties_.Get(CurrentWFps));

  initialCondition->SetPRadpsIC(properties_.Get(CurrentPRadPerSec));
  initialCondition->SetQRadpsIC(properties_.Get(CurrentQRadPerSec));
  initialCondition->SetRRadpsIC(properties_.Get(CurrentRRadPerSec));
}

void FlightDynamics::PreparePropulsionForTrim(gnc::TrimMode mode) {
  if (mode == gnc::TrimMode::Ground) {
    return;
  }

  properties_.Set(SetAllEnginesRunning, -1.0);
}

gnc::TrimResult FlightDynamics::ExecuteTrim(gnc::TrimMode mode) {
  try {
    if (!fdm_->RunIC()) {
      return {
          .success = false,
          .message = "Failed to apply initial conditions.",
      };
    }

    PreparePropulsionForTrim(mode);

    const int jsbMode = ToJSBTrimMode(mode);
    fdm_->DoTrim(jsbMode);

    gnc::TrimResult result = BuildTrimResult();
    ApplyTrimResultToControlInput(result);

    return result;
  } catch (const std::exception &e) {
    gnc::TrimResult result{};
    result.success = false;
    result.message = e.what();

    return result;
  }
}

gnc::TrimResult FlightDynamics::BuildTrimResult() const {
  gnc::TrimResult result{};
  result.success = true;
  result.alphaDeg = properties_.GetAlphaDeg();
  result.betaDeg = properties_.GetBetaDeg();
  result.rollDeg = properties_.GetRollDeg();
  result.pitchDeg = properties_.GetPitchDeg();

  result.throttle = flightControls_.GetThrottle();
  result.elevator = flightControls_.GetElevator();
  result.pitchTrim = flightControls_.GetPitchTrim();
  result.aileron = flightControls_.GetAileron();
  result.rudder = flightControls_.GetRudder();

  result.uDot = properties_.GetUDotMps2();
  result.vDot = properties_.GetVDotMps2();
  result.wDot = properties_.GetWDotMps2();
  result.pDot = properties_.GetPdotDegPerSec2();
  result.qDot = properties_.GetQdotDegPerSec2();
  result.rDot = properties_.GetRdotDegPerSec2();

  return result;
}

void FlightDynamics::ApplyTrimResultToControlInput(
    const gnc::TrimResult &result) {
  controlInput_.elevator = result.elevator;
  flightControls_.SetPitchTrim(result.pitchTrim);
  controlInput_.aileron = result.aileron;
  controlInput_.rudder = result.rudder;
  controlInput_.throttle = result.throttle;
}

void FlightDynamics::ConfigurePaths() {
  fdm_->SetRootDir(SGPath(JSBSIM_ROOT_PATH));
  fdm_->SetAircraftPath(SGPath("aircraft"));
  fdm_->SetEnginePath(SGPath("engine"));
  fdm_->SetSystemsPath(SGPath("systems"));
}

bool FlightDynamics::LoadAircraft(const SimulationConfig &config) {
  if (!fdm_->LoadModel(config.aircraftName)) {
    std::cerr << "Failed to load " << config.aircraftName << '\n';
    return false;
  }

  std::cout << config.aircraftName << " loaded\n";
  return true;
}

void FlightDynamics::ConfigureSimulation(const SimulationConfig &config) {
  fdm_->Setdt(config.GetDT());
}

void FlightDynamics::ConfigureInitialConditions(
    const SimulationConfig &config) {
  auto ic = fdm_->GetIC();

  ic->SetAltitudeASLFtIC(config.altitudeFt);
  ic->SetVcalibratedKtsIC(config.calibratedAirspeedKts);

  ic->SetPhiDegIC(config.rollDeg);
  ic->SetThetaDegIC(config.pitchDeg);
  ic->SetPsiDegIC(config.headingDeg);
}

bool FlightDynamics::InitializeState() {
  if (!fdm_->RunIC()) {
    std::cerr << "Failed to initialize simulation\n";
    return false;
  }

  std::cout << "Initial altitude: " << properties_.GetAltitudeAglFt()
            << " ft\n";
  return true;
}

void FlightDynamics::ApplyControlInput() {
  flightControls_.SetElevator(controlInput_.elevator);
  flightControls_.SetAileron(controlInput_.aileron);
  flightControls_.SetRudder(controlInput_.rudder);
  flightControls_.SetThrottle(controlInput_.throttle);
}
} // namespace sim
