#include "simulation/Aircraft.hpp"

#include "control/ManualControlInputStrategy.hpp"

#include <FGFDMExec.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <initialization/FGInitialCondition.h>
#include <iostream>
#include <models/FGFCS.h>
#include <models/FGPropulsion.h>
#include <models/propulsion/FGEngine.h>
#include <models/propulsion/FGThruster.h>
#include <simgear/misc/sg_path.hxx>
#include <string>
#include <utility>

namespace {
constexpr const char *CurrentAltitudeAslFt = "position/h-sl-ft";
constexpr const char *CurrentLatitudeRad = "position/lat-gc-rad";
constexpr const char *CurrentLongitudeRad = "position/long-gc-rad";
constexpr const char *CurrentRollRad = "attitude/phi-rad";
constexpr const char *CurrentPitchRad = "attitude/theta-rad";
constexpr const char *CurrentHeadingRad = "attitude/psi-rad";
constexpr const char *CurrentPRadPerSec = "velocities/p-rad_sec";
constexpr const char *CurrentQRadPerSec = "velocities/q-rad_sec";
constexpr const char *CurrentRRadPerSec = "velocities/r-rad_sec";

constexpr double RadToDeg = 57.295779513082320876;

double RadToDegValue(double value) { return value * RadToDeg; }

bool IsFiniteInitialCondition(const sim::InitialCondition &initialCondition) {
  return std::isfinite(initialCondition.latitudeDeg)
         && std::isfinite(initialCondition.longitudeDeg)
         && std::isfinite(initialCondition.altitudeFt)
         && std::isfinite(initialCondition.rollDeg)
         && std::isfinite(initialCondition.pitchDeg)
         && std::isfinite(initialCondition.headingDeg)
         && std::isfinite(initialCondition.airspeedKts)
         && std::isfinite(initialCondition.pRadPerSec)
         && std::isfinite(initialCondition.qRadPerSec)
         && std::isfinite(initialCondition.rRadPerSec);
}

std::string ResolveJSBSimRootPath() {
  const std::filesystem::path cwd = std::filesystem::current_path();
  const std::filesystem::path candidates[] = {
      cwd / "build" / "debug" / "_deps" / "jsbsim-src",
      cwd / "_deps" / "jsbsim-src",
      cwd.parent_path() / "_deps" / "jsbsim-src",
  };

  for (const auto &candidate : candidates) {
    if (std::filesystem::exists(candidate / "aircraft")
        && std::filesystem::exists(candidate / "engine")) {
      return candidate.generic_string();
    }
  }

  return (cwd / "build" / "debug" / "_deps" / "jsbsim-src").generic_string();
}
} // namespace

namespace sim {
Aircraft::Aircraft()
    : fdm_(std::make_unique<JSBSim::FGFDMExec>()), properties_(*fdm_),
      flightControls_(*fdm_),
      controlInputStrategy_(
          std::make_unique<control::ManualControlInputStrategy>()) {}

Aircraft::~Aircraft() = default;

bool Aircraft::Initialize(const SimulationConfig &config) {
  controlInput_ = {};
  controlInputStrategy_->Reset();

  ConfigurePaths();
  if (!LoadAircraft(config)) {
    return false;
  }

  ConfigureSimulation(config);
  ConfigureInitialConditions(config);
  return InitializeState();
}

bool Aircraft::Step() {
  UpdateControlInput();
  ApplyControlInput();
  return fdm_->Run();
}

bool Aircraft::Update() { return Step(); }

AircraftState Aircraft::GetAircraftState() const {
  AircraftState state{};
  state.simulationTimeSec = properties_.GetSimTimeSec();
  state.altitudeAglFt = properties_.GetAltitudeAglFt();
  state.calibratedAirspeedKts = properties_.GetCalibratedAirspeedKts();
  state.trueAirspeedMps = properties_.GetTrueAirspeedMps();
  state.rollDeg = properties_.GetRollDeg();
  state.pitchDeg = properties_.GetPitchDeg();
  state.alphaDeg = properties_.GetAlphaDeg();
  state.betaDeg = properties_.GetBetaDeg();
  state.uMps = properties_.GetUMps();
  state.vMps = properties_.GetVMps();
  state.wMps = properties_.GetWMps();
  state.pDegPerSec = properties_.GetPDegPerSec();
  state.qDegPerSec = properties_.GetQDegPerSec();
  state.rDegPerSec = properties_.GetRDegPerSec();
  return state;
}

AircraftStateDerivative Aircraft::GetAircraftStateDerivative() const {
  AircraftStateDerivative derivative{};
  derivative.uDotMps2 = properties_.GetUDotMps2();
  derivative.vDotMps2 = properties_.GetVDotMps2();
  derivative.wDotMps2 = properties_.GetWDotMps2();
  derivative.pDotDegPerSec2 = properties_.GetPdotDegPerSec2();
  derivative.qDotDegPerSec2 = properties_.GetQdotDegPerSec2();
  derivative.rDotDegPerSec2 = properties_.GetRdotDegPerSec2();
  return derivative;
}

bool Aircraft::ApplyInitialCondition(const InitialCondition &initialCondition) {
  if (!IsFiniteInitialCondition(initialCondition)) {
    return false;
  }

  SetInitialConditionInputs(initialCondition);
  return fdm_->RunIC();
}

void Aircraft::SetInitialConditionInputs(
    const InitialCondition &initialCondition) {
  auto ic = fdm_->GetIC();

  ic->SetLatitudeDegIC(initialCondition.latitudeDeg);
  ic->SetLongitudeDegIC(initialCondition.longitudeDeg);
  ic->SetAltitudeASLFtIC(initialCondition.altitudeFt);

  ic->SetPhiDegIC(initialCondition.rollDeg);
  ic->SetThetaDegIC(initialCondition.pitchDeg);
  ic->SetPsiDegIC(initialCondition.headingDeg);

  ic->SetVcalibratedKtsIC(initialCondition.airspeedKts);

  ic->SetPRadpsIC(initialCondition.pRadPerSec);
  ic->SetQRadpsIC(initialCondition.qRadPerSec);
  ic->SetRRadpsIC(initialCondition.rRadPerSec);
}

InitialCondition Aircraft::CaptureCurrentCondition() const {
  InitialCondition initialCondition{};
  initialCondition.latitudeDeg =
      RadToDegValue(properties_.Get(CurrentLatitudeRad));
  initialCondition.longitudeDeg =
      RadToDegValue(properties_.Get(CurrentLongitudeRad));
  initialCondition.altitudeFt = properties_.Get(CurrentAltitudeAslFt);
  initialCondition.rollDeg = RadToDegValue(properties_.Get(CurrentRollRad));
  initialCondition.pitchDeg = RadToDegValue(properties_.Get(CurrentPitchRad));
  initialCondition.headingDeg =
      RadToDegValue(properties_.Get(CurrentHeadingRad));
  initialCondition.airspeedKts = properties_.GetCalibratedAirspeedKts();
  initialCondition.pRadPerSec = properties_.Get(CurrentPRadPerSec);
  initialCondition.qRadPerSec = properties_.Get(CurrentQRadPerSec);
  initialCondition.rRadPerSec = properties_.Get(CurrentRRadPerSec);

  return initialCondition;
}

bool Aircraft::Reset(const SimulationConfig &config,
    const InitialCondition &initialCondition) {
  ConfigureSimulation(config);
  ResetControlInput();

  if (!ApplyInitialCondition(initialCondition)) {
    return false;
  }

  fdm_->Setsim_time(0.0);

  ResetControlInput();
  return true;
}

JSBSim::FGFDMExec &Aircraft::GetFDMExec() { return *fdm_; }

const JSBSim::FGFDMExec &Aircraft::GetFDMExec() const { return *fdm_; }

JSBSim::FlightProperties &Aircraft::GetProperties() { return properties_; }

const JSBSim::FlightProperties &Aircraft::GetProperties() const {
  return properties_;
}

JSBSim::FlightControls &Aircraft::GetFlightControls() {
  return flightControls_;
}

const JSBSim::FlightControls &Aircraft::GetFlightControls() const {
  return flightControls_;
}

const control::ControlInput &Aircraft::GetAircraftControlInput() const {
  return controlInput_;
}

void Aircraft::SetAircraftControlInput(const control::ControlInput &input) {
  SetControlInputCommand(input);
  controlInput_ = controlInputStrategy_->GetCommandedInput();
  control::ClampControlInput(controlInput_);
  ApplyControlInput();
}

const control::ControlInput &Aircraft::GetControlInput() const {
  return GetAircraftControlInput();
}

control::ControlInputStrategy &Aircraft::GetControlInputStrategy() {
  return *controlInputStrategy_;
}

const control::ControlInputStrategy &Aircraft::GetControlInputStrategy() const {
  return *controlInputStrategy_;
}

void Aircraft::SetControlInputStrategy(
    std::unique_ptr<control::ControlInputStrategy> strategy) {
  if (strategy == nullptr) {
    return;
  }

  const control::ControlInput previousInput =
      controlInputStrategy_ != nullptr
          ? controlInputStrategy_->GetCommandedInput()
          : controlInput_;
  controlInputStrategy_ = std::move(strategy);
  SetControlInputCommand(previousInput);
}

bool Aircraft::SetControlInputCommand(const control::ControlInput &input) {
  bool changed = false;
  changed =
      SetControlInputCommand(control::ControlAxis::Elevator, input.elevator)
      || changed;
  changed = SetControlInputCommand(control::ControlAxis::Aileron, input.aileron)
            || changed;
  changed = SetControlInputCommand(control::ControlAxis::Rudder, input.rudder)
            || changed;
  changed =
      SetControlInputCommand(control::ControlAxis::Throttle, input.throttle)
      || changed;
  return changed;
}

bool Aircraft::SetControlInputCommand(control::ControlAxis axis, double value) {
  return controlInputStrategy_->SetCommandedInput(axis, value);
}

bool Aircraft::AdjustControlInputCommand(control::ControlAxis axis,
    double delta) {
  return controlInputStrategy_->AdjustCommandedInput(axis, delta);
}

bool Aircraft::SetElevatorInput(double value) {
  return SetControlInputCommand(control::ControlAxis::Elevator, value);
}

bool Aircraft::SetAileronInput(double value) {
  return SetControlInputCommand(control::ControlAxis::Aileron, value);
}

bool Aircraft::SetRudderInput(double value) {
  return SetControlInputCommand(control::ControlAxis::Rudder, value);
}

bool Aircraft::SetThrottleInput(double value) {
  return SetControlInputCommand(control::ControlAxis::Throttle, value);
}

void Aircraft::ResetControlInput() {
  controlInput_ = {};
  controlInputStrategy_->Reset();

  flightControls_.SetPitchTrim(0.0);
  ApplyControlInput();
}

std::size_t Aircraft::GetEngineCount() const {
  const auto propulsion = fdm_->GetPropulsion();
  return propulsion != nullptr ? propulsion->GetNumEngines() : 0U;
}

EngineState Aircraft::GetEngineState(std::size_t index) const {
  EngineState state{};
  state.index = index;

  const auto propulsion = fdm_->GetPropulsion();
  const auto fcs = fdm_->GetFCS();
  if (propulsion == nullptr || index >= propulsion->GetNumEngines()) {
    return state;
  }

  const auto engine = propulsion->GetEngine(static_cast<unsigned int>(index));
  if (engine == nullptr) {
    return state;
  }

  state.running = engine->GetRunning();
  state.throttleCommand =
      fcs != nullptr ? fcs->GetThrottleCmd(static_cast<int>(index)) : 0.0;

  const auto thruster = engine->GetThruster();
  state.rpm = thruster != nullptr ? thruster->GetEngineRPM() : 0.0;

  return state;
}

std::vector<EngineState> Aircraft::GetEngineStates() const {
  std::vector<EngineState> states;
  const std::size_t engineCount = GetEngineCount();
  states.reserve(engineCount);

  for (std::size_t index = 0; index < engineCount; ++index) {
    states.push_back(GetEngineState(index));
  }

  return states;
}

bool Aircraft::IsAnyEngineRunning() const {
  for (const EngineState &engineState : GetEngineStates()) {
    if (engineState.running) {
      return true;
    }
  }

  return false;
}

bool Aircraft::AreAllEnginesRunning() const {
  const auto engineStates = GetEngineStates();
  if (engineStates.empty()) {
    return false;
  }

  return std::all_of(engineStates.begin(),
      engineStates.end(),
      [](const EngineState &engineState) { return engineState.running; });
}

void Aircraft::ConfigurePaths() {
  fdm_->SetRootDir(SGPath(ResolveJSBSimRootPath()));
  fdm_->SetAircraftPath(SGPath("aircraft"));
  fdm_->SetEnginePath(SGPath("engine"));
  fdm_->SetSystemsPath(SGPath("systems"));
}

bool Aircraft::LoadAircraft(const SimulationConfig &config) {
  if (!fdm_->LoadModel(config.aircraftName)) {
    std::cerr << "Failed to load " << config.aircraftName << '\n';
    return false;
  }

  std::cout << config.aircraftName << " loaded\n";
  return true;
}

void Aircraft::ConfigureSimulation(const SimulationConfig &config) {
  controlDt_ = config.GetDT();
  fdm_->Setdt(config.GetDT());
}

void Aircraft::ConfigureInitialConditions(const SimulationConfig &config) {
  auto ic = fdm_->GetIC();

  ic->SetAltitudeASLFtIC(config.altitudeFt);
  ic->SetVcalibratedKtsIC(config.calibratedAirspeedKts);

  ic->SetPhiDegIC(config.rollDeg);
  ic->SetThetaDegIC(config.pitchDeg);
  ic->SetPsiDegIC(config.headingDeg);
}

bool Aircraft::InitializeState() {
  if (!fdm_->RunIC()) {
    std::cerr << "Failed to initialize simulation\n";
    return false;
  }

  std::cout << "Initial altitude: " << properties_.GetAltitudeAglFt()
            << " ft\n";
  return true;
}

void Aircraft::UseManualControlInputStrategy() {
  SetControlInputStrategy(
      std::make_unique<control::ManualControlInputStrategy>());
}

bool Aircraft::UpdateControlInput() {
  const bool changed =
      controlInputStrategy_->Update(*this, controlDt_, controlInput_);
  control::ClampControlInput(controlInput_);
  return changed;
}

void Aircraft::ApplyControlInput() {
  flightControls_.SetElevator(controlInput_.elevator);
  flightControls_.SetAileron(controlInput_.aileron);
  flightControls_.SetRudder(controlInput_.rudder);
  flightControls_.SetThrottle(controlInput_.throttle);
}
} // namespace sim
