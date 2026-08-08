#include "application/sim/Aircraft.hpp"
#include "application/sim/ErrorTracker.hpp"
#include "application/sim/Simulation.hpp"
#include "application/sim/StateLogger.hpp"
#include "application/sim/gnc/ControlContext.hpp"
#include "application/sim/gnc/hold/CourseHoldController.hpp"
#include "application/sim/gnc/hold/AltitudeHoldController.hpp"
#include "application/sim/control/FlightControlManager.hpp"
#include "application/sim/control/FlightControlMode.hpp"
#include "common/math/Math.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
constexpr double SimTimeTolerance = 1.0e-9;
constexpr double AltitudeToleranceFt = 1.0;
constexpr double AirspeedToleranceKts = 0.5;
constexpr double HeadingToleranceDeg = 0.5;
constexpr double TrimInputTolerance = 1.0e-5;
constexpr double ControlCommandTolerance = 1.0e-6;
constexpr double MaximumAsyncKickoffSec = 1.0;
constexpr double ExpectedLinearizationRefreshIntervalSec = 5.0;

void Require(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void RequireNear(double actual, double expected, double tolerance,
    const std::string &message) {
  if (std::fabs(actual - expected) > tolerance) {
    throw std::runtime_error(message + " actual=" + std::to_string(actual)
                             + " expected=" + std::to_string(expected));
  }
}

template <std::size_t Size>
void RequireArrayNear(const std::array<double, Size> &actual,
    const std::array<double, Size> &expected, double tolerance,
    const std::string &message) {
  for (std::size_t index = 0; index < Size; ++index) {
    RequireNear(actual[index],
        expected[index],
        tolerance,
        message + "[" + std::to_string(index) + "]");
  }
}

void RequireVectorNear(const std::vector<double> &actual,
    const std::vector<double> &expected, double tolerance,
    const std::string &message) {
  Require(actual.size() == expected.size(), message + " size mismatch");
  for (std::size_t index = 0; index < actual.size(); ++index) {
    RequireNear(actual[index],
        expected[index],
        tolerance,
        message + "[" + std::to_string(index) + "]");
  }
}

void RequireKinematicStateNear(const sim::FDMKinematicState &actual,
    const sim::FDMKinematicState &expected, const std::string &message) {
  RequireNear(actual.latitudeRad,
      expected.latitudeRad,
      SimTimeTolerance,
      message + " latitude mismatch");
  RequireNear(actual.longitudeRad,
      expected.longitudeRad,
      SimTimeTolerance,
      message + " longitude mismatch");
  RequireNear(actual.altitudeAslFt,
      expected.altitudeAslFt,
      SimTimeTolerance,
      message + " altitude mismatch");
  RequireArrayNear(actual.bodyVelocityFps,
      expected.bodyVelocityFps,
      SimTimeTolerance,
      message + " body velocity mismatch");
  RequireArrayNear(actual.attitudeRad,
      expected.attitudeRad,
      SimTimeTolerance,
      message + " attitude mismatch");
  RequireArrayNear(actual.bodyAngularRatesRadPerSec,
      expected.bodyAngularRatesRadPerSec,
      SimTimeTolerance,
      message + " angular rate mismatch");
}

void RequireControlStateNear(const sim::FDMControlState &actual,
    const sim::FDMControlState &expected, const std::string &message) {
  RequireNear(actual.elevatorCommand,
      expected.elevatorCommand,
      SimTimeTolerance,
      message + " elevator command mismatch");
  RequireNear(actual.aileronCommand,
      expected.aileronCommand,
      SimTimeTolerance,
      message + " aileron command mismatch");
  RequireNear(actual.rudderCommand,
      expected.rudderCommand,
      SimTimeTolerance,
      message + " rudder command mismatch");
  RequireVectorNear(actual.throttleCommands,
      expected.throttleCommands,
      SimTimeTolerance,
      message + " throttle command mismatch");
  RequireNear(actual.pitchTrimCommand,
      expected.pitchTrimCommand,
      SimTimeTolerance,
      message + " pitch trim mismatch");
  RequireNear(actual.elevatorPositionRad,
      expected.elevatorPositionRad,
      SimTimeTolerance,
      message + " elevator position mismatch");
  RequireNear(actual.leftAileronPositionRad,
      expected.leftAileronPositionRad,
      SimTimeTolerance,
      message + " left aileron position mismatch");
  RequireNear(actual.rightAileronPositionRad,
      expected.rightAileronPositionRad,
      SimTimeTolerance,
      message + " right aileron position mismatch");
  RequireNear(actual.rudderPositionRad,
      expected.rudderPositionRad,
      SimTimeTolerance,
      message + " rudder position mismatch");
  RequireVectorNear(actual.throttlePositions,
      expected.throttlePositions,
      SimTimeTolerance,
      message + " throttle position mismatch");
}

sim::SimulationConfig MakeConfig() {
  sim::SimulationConfig config{};
  config.simulationHz = 120.0;
  return config;
}

void StartSimulation(sim::Simulation &simulation) {
  Require(simulation.Initialize(MakeConfig()),
      "Simulation failed to initialize");
}

double GetSimTime(const sim::Simulation &simulation) {
  return simulation.GetAircraft().GetAircraftState().simulationTimeSec;
}

sim::Tick MakeTestTick(const sim::Simulation &simulation) {
  return {0U, simulation.GetTickSizeSec(), GetSimTime(simulation)};
}

control::FlightControlManager &GetFlightControlManager(
    sim::Simulation &simulation) {
  auto *flightControlManager =
      simulation.GetComponent<control::FlightControlManager>();
  Require(flightControlManager != nullptr,
      "Simulation does not contain FlightControlManager");
  return *flightControlManager;
}

double ComputeRollProportionalGain(const gnc::RollHoldSettings &settings,
    const gnc::RollDynamics &dynamics) {
  const double naturalFrequency = settings.naturalFrequencyRadPerSec;
  return naturalFrequency * naturalFrequency / dynamics.aPhi2;
}

double ComputeRollDerivativeGain(const gnc::RollHoldSettings &settings,
    const gnc::RollDynamics &dynamics) {
  return (2.0 * settings.dampingRatio * settings.naturalFrequencyRadPerSec
             - dynamics.aPhi1)
         / dynamics.aPhi2;
}

double ComputePitchProportionalGain(const gnc::PitchHoldSettings &settings,
    const gnc::PitchDynamics &dynamics) {
  const double naturalFrequency = settings.naturalFrequencyRadPerSec;
  return (naturalFrequency * naturalFrequency - dynamics.aTheta2)
         / dynamics.aTheta3;
}

double ComputePitchDerivativeGain(const gnc::PitchHoldSettings &settings,
    const gnc::PitchDynamics &dynamics) {
  return (2.0 * settings.dampingRatio * settings.naturalFrequencyRadPerSec
             - dynamics.aTheta1)
         / dynamics.aTheta3;
}

void WaitForAutopilotDynamics(sim::Simulation &simulation,
    gnc::Autopilot &autopilot, bool waitForRoll, bool waitForPitch) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(15);
  for (;;) {
    const bool rollReady = !waitForRoll || autopilot.GetRollDynamics();
    const bool pitchReady = !waitForPitch || autopilot.GetPitchDynamics();
    if (rollReady && pitchReady) {
      return;
    }

    Require(std::chrono::steady_clock::now() < deadline,
        "Timed out waiting for asynchronous autopilot dynamics");
    Require(simulation.Tick(),
        "Simulation tick failed while waiting for autopilot dynamics");
    std::this_thread::yield();
  }
}

void WaitForLinearizationResult(sim::Simulation &simulation,
    gnc::Autopilot &autopilot) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (autopilot.GetLinearizationResult() == nullptr) {
    Require(std::chrono::steady_clock::now() < deadline,
        "Timed out waiting for asynchronous linearization");
    Require(simulation.Tick(),
        "Simulation tick failed while waiting for linearization");
    std::this_thread::yield();
  }
}

struct ComponentLifecycleCounts {
  int initialize = 0;
  int reset = 0;
  int preTick = 0;
  int tick = 0;
  int postTick = 0;
  int shutdown = 0;
};

class LifecycleTestComponent final : public sim::Component {
public:
  explicit LifecycleTestComponent(ComponentLifecycleCounts &counts)
      : counts_(counts) {}

protected:
  bool OnInitialize() override {
    ++counts_.initialize;
    return true;
  }
  bool OnReset() override {
    ++counts_.reset;
    return true;
  }
  bool OnPreTick(const sim::Tick &) override {
    ++counts_.preTick;
    return true;
  }
  bool OnTick(const sim::Tick &) override {
    ++counts_.tick;
    return true;
  }
  bool OnPostTick(const sim::Tick &) override {
    ++counts_.postTick;
    return true;
  }
  void OnShutdown() override { ++counts_.shutdown; }

private:
  ComponentLifecycleCounts &counts_;
};

class ComponentLookupTestComponent final : public sim::Component {
public:
  bool FoundLifecycleComponent() const { return foundLifecycleComponent_; }
  bool HasAircraftAccess() const { return hasAircraftAccess_; }

protected:
  bool OnInitialize() override {
    foundLifecycleComponent_ =
        GetComponent<LifecycleTestComponent>() != nullptr;
    hasAircraftAccess_ =
        std::isfinite(GetAircraft().GetAircraftState().simulationTimeSec);
    return foundLifecycleComponent_ && hasAircraftAccess_;
  }

private:
  bool foundLifecycleComponent_ = false;
  bool hasAircraftAccess_ = false;
};

class RegistryTestController final : public gnc::Controller {
public:
  explicit RegistryTestController(int &resetCount) : resetCount_(resetCount) {}

  void Reset() override { ++resetCount_; }

private:
  int &resetCount_;
};

void TestErrorTrackerOwnsErrorState() {
  sim::ErrorTracker errorTracker;
  Require(!errorTracker.HasError(), "New error tracker contains an error");

  errorTracker.SetError("specific error");
  errorTracker.SetErrorIfEmpty("fallback error");
  Require(errorTracker.GetLastError() == "specific error",
      "Fallback replaced a specific error");

  errorTracker.ClearError();
  Require(!errorTracker.HasError(), "Error tracker did not clear its error");

  errorTracker.SetErrorIfEmpty("fallback error");
  Require(errorTracker.GetLastError() == "fallback error",
      "Fallback error was not stored");
}

void TestSimulationComponentLifecycle() {
  sim::Simulation simulation;
  ComponentLifecycleCounts counts;
  auto *component = simulation.AddComponent<LifecycleTestComponent>(counts);
  auto *lookup = simulation.AddComponent<ComponentLookupTestComponent>();

  Require(component != nullptr, "Failed to add lifecycle test component");
  Require(lookup != nullptr, "Failed to add component lookup test component");
  Require(simulation.GetComponent<sim::StateLogger>() != nullptr,
      "Simulation does not contain StateLogger");
  Require(counts.initialize == 0,
      "Component initialized before Simulation initialization");
  Require(simulation.GetComponent<LifecycleTestComponent>() == component,
      "GetComponent did not return the added component");

  StartSimulation(simulation);
  Require(counts.initialize == 1, "Component was not initialized");
  Require(lookup->FoundLifecycleComponent(),
      "Component could not find another component through its owner");
  Require(lookup->HasAircraftAccess(),
      "Component could not access Aircraft through its protected helper");

  const sim::Simulation &constSimulation = simulation;
  Require(constSimulation.GetComponent<ComponentLookupTestComponent>()
              == lookup,
      "Const GetComponent did not return the added component");

  Require(simulation.Tick(), "Component lifecycle tick failed");
  Require(counts.preTick == 1 && counts.tick == 1 && counts.postTick == 1,
      "Component tick lifecycle hooks were not called");

  Require(simulation.Reset(), "Component lifecycle reset failed");
  Require(counts.reset == 1, "Component reset hook was not called");

  Require(simulation.RemoveComponent<LifecycleTestComponent>(),
      "Failed to remove lifecycle test component");
  Require(counts.shutdown == 1,
      "Removing a component did not run its shutdown hook");
  Require(simulation.GetComponent<LifecycleTestComponent>() == nullptr,
      "Removed component is still accessible");

  ComponentLifecycleCounts lateCounts;
  auto *lateComponent =
      simulation.AddComponent<LifecycleTestComponent>(lateCounts);
  Require(lateComponent != nullptr, "Failed to add late component");
  Require(lateCounts.initialize == 1,
      "Component added after initialization was not initialized immediately");
  Require(simulation.RemoveComponent<LifecycleTestComponent>(),
      "Failed to remove late component");
  Require(lateCounts.shutdown == 1,
      "Late component shutdown hook was not called");

  simulation.Shutdown();
}

void TestTickAdvancesOneStep() {
  sim::Simulation simulation;
  StartSimulation(simulation);

  const double startTime = GetSimTime(simulation);
  Require(simulation.Tick(), "Simulation tick failed");
  RequireNear(GetSimTime(simulation),
      startTime + simulation.GetTickSizeSec(),
      SimTimeTolerance,
      "Tick did not advance exactly one simulation step");
}

void TestResetUsesDefaultInitialCondition() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  for (int i = 0; i < 3; ++i) {
    Require(simulation.Tick(), "Pre-reset tick failed");
  }

  Require(simulation.Reset(), "Reset failed");

  RequireNear(GetSimTime(simulation),
      0.0,
      SimTimeTolerance,
      "Reset did not reset simulation time");

  const sim::InitialCondition captured = simulation.GetCurrentCondition();
  const sim::InitialCondition &defaultInitialCondition =
      simulation.GetDefaultInitialCondition();
  RequireNear(captured.altitudeFt,
      defaultInitialCondition.altitudeFt,
      AltitudeToleranceFt,
      "Reset altitude does not match default IC");
  RequireNear(captured.airspeedKts,
      defaultInitialCondition.airspeedKts,
      AirspeedToleranceKts,
      "Reset airspeed does not match default IC");
}

void TestResetWithInitialCondition() {
  sim::Simulation simulation;
  StartSimulation(simulation);

  sim::InitialCondition initialCondition =
      simulation.GetDefaultInitialCondition();
  initialCondition.altitudeFt = 2500.0;
  initialCondition.headingDeg = 45.0;
  initialCondition.airspeedKts = 95.0;

  Require(simulation.Reset(initialCondition), "Reset with custom IC failed");

  const sim::InitialCondition captured = simulation.GetCurrentCondition();
  RequireNear(captured.altitudeFt,
      initialCondition.altitudeFt,
      AltitudeToleranceFt,
      "Custom reset altitude mismatch");
  RequireNear(captured.headingDeg,
      initialCondition.headingDeg,
      HeadingToleranceDeg,
      "Custom reset heading mismatch");
  RequireNear(captured.airspeedKts,
      initialCondition.airspeedKts,
      AirspeedToleranceKts,
      "Custom reset airspeed mismatch");

  Require(simulation.Reset(), "Default reset after custom reset failed");
  const sim::InitialCondition restoredDefault =
      simulation.GetCurrentCondition();
  RequireNear(restoredDefault.altitudeFt,
      simulation.GetDefaultInitialCondition().altitudeFt,
      AltitudeToleranceFt,
      "Custom reset replaced the default altitude");
  RequireNear(restoredDefault.airspeedKts,
      simulation.GetDefaultInitialCondition().airspeedKts,
      AirspeedToleranceKts,
      "Custom reset replaced the default airspeed");
}

void TestCaptureCurrentStateCanReset() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  Require(simulation.Tick(), "Tick before capture failed");

  const sim::InitialCondition captured = simulation.GetCurrentCondition();
  Require(simulation.Reset(captured), "Reset with captured state failed");
  const sim::InitialCondition restored = simulation.GetCurrentCondition();

  RequireNear(restored.altitudeFt,
      captured.altitudeFt,
      AltitudeToleranceFt,
      "Captured reset altitude mismatch");
  RequireNear(restored.headingDeg,
      captured.headingDeg,
      HeadingToleranceDeg,
      "Captured reset heading mismatch");
  RequireNear(restored.airspeedKts,
      captured.airspeedKts,
      AirspeedToleranceKts,
      "Captured reset airspeed mismatch");
}

void TestEngineStateInspection() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  const auto &aircraft = simulation.GetAircraft();
  const auto &engines = aircraft.GetEngines();
  const std::size_t engineCount = engines.GetEngineCount();

  Require(engineCount >= 1, "Expected at least one engine");

  const sim::EngineState engineState = engines.GetEngineState(0);

  Require(engineState.index == 0, "Engine index mismatch");
  if (engineCount == 1) {
    Require(engineState.running == engines.IsAnyEngineRunning(),
        "Single engine running state differs from aggregate query");
    Require(engineState.running == engines.AreAllEnginesRunning(),
        "Single engine running state differs from all-engines query");
  }
  Require(std::isfinite(engineState.rpm), "Engine RPM is not finite");
  Require(std::isfinite(engineState.throttleCommand),
      "Engine throttle command is not finite");

  const sim::EngineState invalidEngineState =
      engines.GetEngineState(engineCount + 1);
  Require(invalidEngineState.index == engineCount + 1,
      "Invalid engine index was not preserved");
}

void TestInvalidInitialConditionFails() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  sim::InitialCondition invalid = simulation.GetDefaultInitialCondition();
  invalid.latitudeDeg = 100.0;

  std::string validationError;
  Require(!sim::ValidateInitialCondition(invalid, &validationError),
      "Standalone validation accepted invalid latitude");
  Require(validationError
              == "Latitude must be finite and between -90 and 90 degrees.",
      "Standalone validation returned a different error");

  Require(!simulation.Reset(invalid), "Invalid latitude was accepted");
  Require(simulation.GetErrorTracker().GetLastError().has_value(),
      "Invalid IC did not report an error");
}

void TestAircraftStateAccess() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  const sim::Aircraft &aircraft = simulation.GetAircraft();
  const sim::AircraftState aircraftState = aircraft.GetAircraftState();
  const sim::AircraftStateDerivative derivative =
      aircraft.GetAircraftStateDerivative();
  const sim::InitialCondition currentCondition = aircraft.GetCurrentCondition();
  const auto &properties = aircraft.GetProperties();

  Require(std::isfinite(currentCondition.altitudeFt),
      "Aircraft altitude invalid");
  Require(std::isfinite(aircraftState.trueAirspeedMps),
      "Aircraft airspeed invalid");
  Require(std::isfinite(aircraftState.alphaDeg),
      "Aircraft state alpha invalid");
  Require(std::isfinite(aircraftState.betaDeg), "Aircraft state beta invalid");
  Require(std::isfinite(derivative.uDotMps2),
      "Aircraft state derivative uDot invalid");
  RequireNear(properties.Roll().Rad(),
      math::DegToRad(aircraftState.rollDeg),
      SimTimeTolerance,
      "Flight properties roll rad mismatch");
  RequireNear(properties.Pitch().Rad(),
      math::DegToRad(aircraftState.pitchDeg),
      SimTimeTolerance,
      "Flight properties pitch rad mismatch");
  RequireNear(properties.P().RadPerSec(),
      math::DegToRad(aircraftState.pDegPerSec),
      SimTimeTolerance,
      "Flight properties roll rate rad mismatch");
  RequireNear(properties.P().DotRadPerSec2(),
      math::DegToRad(derivative.pDotDegPerSec2),
      SimTimeTolerance,
      "Flight properties roll acceleration rad mismatch");
  RequireNear(properties.TrueAirspeed().Mps(),
      aircraftState.trueAirspeedMps,
      SimTimeTolerance,
      "Flight properties true airspeed mps mismatch");
  RequireNear(properties.AltitudeAgl().Ft(),
      aircraftState.altitudeAglFt,
      SimTimeTolerance,
      "Aircraft state AGL altitude mismatch");
  RequireNear(properties.U().Mps(),
      aircraftState.uMps,
      SimTimeTolerance,
      "Flight properties U mps mismatch");
  RequireNear(properties.U().DotMps2(),
      derivative.uDotMps2,
      SimTimeTolerance,
      "Flight properties U acceleration mismatch");
  RequireNear(properties.VerticalSpeed().FtPerMin(),
      properties.VerticalSpeed().Fps() * 60.0,
      SimTimeTolerance,
      "Flight properties vertical speed ft/min mismatch");
  Require(std::isfinite(properties.Alpha().Rad()),
      "Flight properties alpha rad invalid");
  Require(std::isfinite(properties.Beta().Rad()),
      "Flight properties beta rad invalid");
}

void TestNavigationProperties() {
  sim::Aircraft aircraft;
  sim::InitialCondition initialCondition{};
  initialCondition.headingDeg = 0.0;
  initialCondition.airspeedKts = 100.0;
  Require(aircraft.Initialize(MakeConfig(), initialCondition),
      "Aircraft failed to initialize for navigation property test");

  sim::FDMState state = aircraft.ExtractFDMState(sim::FDMStateFlags::State);
  state.state.bodyVelocityFps = {120.0, 80.0, 0.0};
  state.state.attitudeRad = {0.0, 0.0, 0.0};
  aircraft.ApplyFDMState(state);
  Require(aircraft.Tick(), "Navigation property test tick failed");

  const sim::jsbsim::Properties &properties = aircraft.GetProperties();
  const double northVelocityFps = properties.NorthVelocity().Fps();
  const double eastVelocityFps = properties.EastVelocity().Fps();
  const double expectedGroundSpeedFps =
      std::hypot(northVelocityFps, eastVelocityFps);
  RequireNear(properties.GroundSpeed().Fps(),
      expectedGroundSpeedFps,
      SimTimeTolerance,
      "Ground speed does not match horizontal navigation velocity");
  RequireNear(properties.GroundSpeed().Mps(),
      expectedGroundSpeedFps * 0.3048,
      SimTimeTolerance,
      "Ground speed metric conversion mismatch");

  const double expectedCourseRad =
      std::atan2(eastVelocityFps, northVelocityFps);
  RequireNear(properties.Course().Rad(),
      expectedCourseRad,
      SimTimeTolerance,
      "Course does not match horizontal ground-track direction");
  const sim::AircraftState aircraftState = aircraft.GetAircraftState();
  const double expectedCourseDeg =
      math::Wrap(math::RadToDeg(expectedCourseRad), 0.0, 360.0);
  RequireNear(aircraftState.courseDeg,
      expectedCourseDeg,
      SimTimeTolerance,
      "Aircraft state course is not normalized ground track");
  const double headingRad = math::DegToRad(aircraftState.headingDeg);
  Require(std::fabs(properties.Course().Rad() - headingRad) > 0.1,
      "Course accessor returned aircraft heading instead of ground track");

  const double gravityMps2 = properties.GravityMps2();
  Require(gravityMps2 > 8.0 && gravityMps2 < 11.0,
      "Local gravitational acceleration is not physically reasonable");
}

void TestStartAppliesInitialTrim() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  const auto &aircraft = simulation.GetAircraft();
  const auto &controls = aircraft.GetControls();
  const control::ControlInput &input = controls.GetInput();
  const double pitchTrim = controls.GetPitchTrim();

  Require(input.throttle > TrimInputTolerance,
      "Initial trim did not apply throttle command input");
  Require(std::fabs(pitchTrim) > TrimInputTolerance,
      "Initial trim did not apply pitch trim");
  RequireNear(GetSimTime(simulation),
      0.0,
      SimTimeTolerance,
      "Initial trim changed simulation time");
}

void TestInitialTrimIsStoredInAutopilot() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  const auto &autopilot = GetFlightControlManager(simulation).GetAutopilot();
  const gnc::TrimResult *trimResult = autopilot.GetTrimResult();

  Require(trimResult != nullptr, "Autopilot did not store initial trim result");
  Require(trimResult->success, "Autopilot stored a failed initial trim result");
}

void TestAutopilotControllerRegistry() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  auto &autopilot = GetFlightControlManager(simulation).GetAutopilot();

  Require(autopilot.GetController<gnc::RollHoldController>() != nullptr,
      "Autopilot is missing RollHoldController");
  Require(autopilot.GetController<gnc::PitchHoldController>() != nullptr,
      "Autopilot is missing PitchHoldController");
  Require(autopilot.GetController<gnc::AirspeedHoldController>() != nullptr,
      "Autopilot is missing AirspeedHoldController");
  Require(autopilot.GetController<gnc::CourseHoldController>() != nullptr,
      "Autopilot is missing CourseHoldController");
  Require(autopilot.GetController<gnc::AltitudeHoldController>() != nullptr,
      "Autopilot is missing AltitudeHoldController");

  const gnc::Autopilot &constAutopilot = autopilot;
  Require(constAutopilot.GetController<gnc::RollHoldController>() != nullptr,
      "Const controller lookup failed");

  int resetCount = 0;
  auto *controller =
      autopilot.AddController<RegistryTestController>(resetCount);
  Require(controller != nullptr, "Failed to add controller to Autopilot");
  Require(autopilot.GetController<RegistryTestController>() == controller,
      "Controller lookup did not return the registered controller");

  autopilot.OnReset();
  Require(resetCount == 1,
      "Autopilot did not reset a registered controller generically");
  Require(autopilot.RemoveController<RegistryTestController>(),
      "Failed to remove controller from Autopilot");
  Require(autopilot.GetController<RegistryTestController>() == nullptr,
      "Removed controller is still registered");
}

void TestControlSystemAxisSettersClampFinalInput() {
  sim::Aircraft aircraft;
  auto &controls = aircraft.GetControls();

  Require(controls.SetElevator(-2.0), "Elevator setter did not change");
  Require(controls.SetAileron(2.0), "Aileron setter did not change");
  Require(controls.SetRudder(3.0), "Rudder setter did not change");
  Require(controls.SetThrottle(0.5), "Throttle setter did not change");
  Require(controls.SetThrottle(-1.0), "Throttle setter did not change");

  const control::ControlInput &input = controls.GetInput();
  RequireNear(input.elevator,
      -1.0,
      SimTimeTolerance,
      "Elevator setter did not clamp lower bound");
  RequireNear(input.aileron,
      1.0,
      SimTimeTolerance,
      "Aileron setter did not clamp upper bound");
  RequireNear(input.rudder,
      1.0,
      SimTimeTolerance,
      "Rudder setter did not clamp upper bound");
  RequireNear(input.throttle,
      0.0,
      SimTimeTolerance,
      "Throttle setter did not clamp lower bound");
}

void TestControlSystemSetInputClampsFinalInput() {
  sim::Aircraft aircraft;
  auto &controls = aircraft.GetControls();

  controls.SetInput({
      .elevator = -2.0,
      .aileron = 2.0,
      .rudder = 3.0,
      .throttle = 2.0,
  });

  const control::ControlInput &input = controls.GetInput();
  RequireNear(input.elevator,
      -1.0,
      SimTimeTolerance,
      "Aircraft input did not clamp elevator lower bound");
  RequireNear(input.aileron,
      1.0,
      SimTimeTolerance,
      "Aircraft input did not clamp aileron upper bound");
  RequireNear(input.rudder,
      1.0,
      SimTimeTolerance,
      "Aircraft input did not clamp rudder upper bound");
  RequireNear(input.throttle,
      1.0,
      SimTimeTolerance,
      "Aircraft input did not clamp throttle upper bound");
}

void TestManualFlightControlControllerAppliesCommands() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  auto &aircraft = simulation.GetAircraft();
  auto &flightControlManager = GetFlightControlManager(simulation);
  auto &manualController = flightControlManager.GetManualController();

  flightControlManager.SetMode(control::FlightControlMode::Manual);
  manualController.SetCommandedInput({
      .elevator = 0.25,
      .aileron = 2.0,
      .rudder = -0.25,
      .throttle = 0.5,
  });

  const control::ControlInput &commandedInput =
      manualController.GetCommandedInput();
  RequireNear(commandedInput.throttle,
      0.5,
      SimTimeTolerance,
      "Throttle command mismatch");
  RequireNear(commandedInput.aileron,
      1.0,
      SimTimeTolerance,
      "Manual controller should clamp commanded aileron");
  RequireNear(commandedInput.elevator,
      0.25,
      SimTimeTolerance,
      "Elevator command mismatch");
  RequireNear(commandedInput.rudder,
      -0.25,
      SimTimeTolerance,
      "Rudder command mismatch");

  Require(simulation.Tick(), "Manual flight control tick failed");

  const control::ControlInput &actualInput = aircraft.GetControls().GetInput();
  RequireNear(actualInput.throttle,
      0.5,
      SimTimeTolerance,
      "Throttle command was not applied");
  RequireNear(actualInput.aileron,
      1.0,
      SimTimeTolerance,
      "Aileron command was not applied");
  RequireNear(actualInput.elevator,
      0.25,
      SimTimeTolerance,
      "Elevator command was not applied");
  RequireNear(actualInput.rudder,
      -0.25,
      SimTimeTolerance,
      "Rudder command was not applied");
}

void TestFlightControlManagerOwnsAndRoutesControllers() {
  const control::ControlInput manualInput{
      .elevator = 0.1,
      .aileron = 0.2,
      .rudder = 0.3,
      .throttle = 0.4,
  };
  sim::Simulation simulation;
  StartSimulation(simulation);
  auto &manager = GetFlightControlManager(simulation);
  const auto &aircraft = simulation.GetAircraft();

  manager.GetManualController().SetCommandedInput(manualInput);
  Require(simulation.Tick(), "Manual manager routing tick failed");
  Require(aircraft.GetControls().GetInput() == manualInput,
      "Manager did not route its manual controller output");

  manager.SetMode(control::FlightControlMode::Autopilot);
  Require(simulation.Tick(), "Autopilot manager routing tick failed");
  Require(aircraft.GetControls().GetInput() == manualInput,
      "Autopilot did not preserve manual pass-through output");
}

void TestFlightControlManagerNoInputPreservesCommand() {
  const control::ControlInput retainedInput{
      .elevator = 0.1,
      .aileron = -0.2,
      .rudder = 0.3,
      .throttle = 0.4,
  };
  sim::Simulation simulation;
  StartSimulation(simulation);
  auto &manager = GetFlightControlManager(simulation);
  auto &controls = simulation.GetAircraft().GetControls();

  controls.SetInput(retainedInput);
  manager.SetMode(control::FlightControlMode::None);
  Require(simulation.Tick(), "No-input manager tick failed");
  Require(controls.GetInput() == retainedInput,
      "No-input mode replaced the existing control command");

  manager.GetManualController().SetCommandedInput({});
  manager.SetMode(control::FlightControlMode::Manual);
  Require(simulation.Tick(), "Explicit-zero manager tick failed");
  Require(controls.GetInput() == control::ControlInput{},
      "Explicit zero command was treated as no control update");
}

void TestManualModeIgnoresAutopilotSource() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  auto &aircraft = simulation.GetAircraft();
  auto &flightControlManager = GetFlightControlManager(simulation);
  auto &manualController = flightControlManager.GetManualController();
  auto &autopilot = flightControlManager.GetAutopilot();

  flightControlManager.SetMode(control::FlightControlMode::Manual);
  manualController.SetCommandedInput({
      .elevator = 0.2,
      .aileron = -0.8,
      .rudder = 0.1,
      .throttle = 0.4,
  });

  const auto &properties = aircraft.GetProperties();
  autopilot.SetRollHoldSettings({
      .targetRollRad = properties.Roll().Rad() + 0.1,
      .dampingRatio = 0.7,
      .naturalFrequencyRadPerSec = 1.0,
  });
  autopilot.SetRollHoldEnabled(true);

  Require(simulation.Tick(), "Manual mode tick failed");

  const control::ControlInput &actualInput = aircraft.GetControls().GetInput();
  RequireNear(actualInput.elevator,
      0.2,
      SimTimeTolerance,
      "Manual mode did not apply manual elevator");
  RequireNear(actualInput.aileron,
      -0.8,
      SimTimeTolerance,
      "Manual mode should ignore autopilot aileron");
  RequireNear(actualInput.rudder,
      0.1,
      SimTimeTolerance,
      "Manual mode did not apply manual rudder");
  RequireNear(actualInput.throttle,
      0.4,
      SimTimeTolerance,
      "Manual mode did not apply manual throttle");
}

void TestLinearizationRunsInManualModeWithoutHolds() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  auto &flightControlManager = GetFlightControlManager(simulation);
  auto &autopilot = flightControlManager.GetAutopilot();
  const control::ControlInput manualInput{
      .elevator = 0.2,
      .aileron = -0.3,
      .rudder = 0.1,
      .throttle = 0.4,
  };

  flightControlManager.SetMode(control::FlightControlMode::Manual);
  flightControlManager.GetManualController().SetCommandedInput(manualInput);
  Require(!autopilot.IsRollHoldEnabled() && !autopilot.IsPitchHoldEnabled()
              && !autopilot.IsCourseHoldEnabled(),
      "Always-on linearization test unexpectedly has an active Hold");

  Require(simulation.Tick(), "Linearization scheduling tick failed");
  const double firstRequestDueSec =
      GetSimTime(simulation) + ExpectedLinearizationRefreshIntervalSec;
  while (GetSimTime(simulation) < firstRequestDueSec) {
    Require(simulation.Tick(),
        "Simulation tick failed before periodic linearization request");
  }

  const auto kickoffStart = std::chrono::steady_clock::now();
  Require(simulation.Tick(), "Manual-mode linearization kickoff tick failed");
  const double kickoffDurationSec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - kickoffStart)
                                        .count();
  Require(kickoffDurationSec < MaximumAsyncKickoffSec,
      "Manual-mode linearization blocked the simulation tick");

  WaitForLinearizationResult(simulation, autopilot);
  Require(autopilot.GetLinearizationResult() != nullptr,
      "Manual mode did not publish an asynchronous linearization result");
  Require(flightControlManager.GetMode() == control::FlightControlMode::Manual,
      "Linearization changed the active flight-control mode");
  Require(simulation.GetAircraft().GetControls().GetInput() == manualInput,
      "Background linearization changed the manual control input");
}

void TestRollHoldControllerComputesAileronCommand() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  auto &aircraft = simulation.GetAircraft();
  auto *rollHold = GetFlightControlManager(simulation)
                       .GetAutopilot()
                       .GetController<gnc::RollHoldController>();
  Require(rollHold != nullptr, "Autopilot is missing RollHoldController");

  const gnc::ControlContext emptyContext{};
  rollHold->SetEnabled(false);
  Require(!rollHold->OnTick(aircraft, MakeTestTick(simulation), emptyContext)
              .has_value(),
      "Disabled roll hold should not produce aileron command");

  const auto &properties = aircraft.GetProperties();
  const gnc::RollHoldSettings settings{
      .targetRollRad = properties.Roll().Rad() + 0.2,
      .dampingRatio = 0.7,
      .naturalFrequencyRadPerSec = 3.0,
  };
  const gnc::ControlContext context{
      .rollDynamics =
          gnc::RollDynamics{
              .aPhi1 = 0.4,
              .aPhi2 = 2.0,
          },
  };
  rollHold->SetTrimAileron(0.1);
  rollHold->SetSettings(settings);
  rollHold->SetEnabled(true);

  const auto command =
      rollHold->OnTick(aircraft, MakeTestTick(simulation), context);
  Require(command.has_value(), "Enabled roll hold produced no command");

  const gnc::RollDynamics &dynamics = *context.rollDynamics;
  const double expectedAileron =
      0.1
      + ComputeRollProportionalGain(settings, dynamics)
            * (settings.targetRollRad - properties.Roll().Rad())
      - ComputeRollDerivativeGain(settings, dynamics)
            * properties.P().RadPerSec();
  RequireNear(*command,
      expectedAileron,
      SimTimeTolerance,
      "Roll hold aileron command mismatch");

  const double commandedRollRad = properties.Roll().Rad() - 0.15;
  rollHold->SetEnabled(false);
  const auto cascadedCommand = rollHold->OnTick(aircraft,
      MakeTestTick(simulation),
      context,
      commandedRollRad);
  Require(cascadedCommand.has_value(),
      "Roll hold rejected an outer-loop roll command");
  const double expectedCascadedAileron =
      0.1
      + ComputeRollProportionalGain(settings, dynamics)
            * (commandedRollRad - properties.Roll().Rad())
      - ComputeRollDerivativeGain(settings, dynamics)
            * properties.P().RadPerSec();
  RequireNear(*cascadedCommand,
      expectedCascadedAileron,
      SimTimeTolerance,
      "Roll hold did not use the outer-loop roll command");
  RequireNear(rollHold->GetSettings().targetRollRad,
      settings.targetRollRad,
      SimTimeTolerance,
      "Outer-loop roll command replaced the standalone Roll Hold target");
}

void TestCourseHoldCommandInterface() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  auto &autopilot = GetFlightControlManager(simulation).GetAutopilot();
  auto *courseHold = autopilot.GetController<gnc::CourseHoldController>();
  Require(courseHold != nullptr, "Autopilot is missing CourseHoldController");

  const gnc::CourseHoldSettings settings{
      .targetCourseRad = 1.2,
      .dampingRatio = 0.8,
      .bandwidthSeparationRatio = 5.0,
  };
  autopilot.SetCourseHoldSettings(settings);
  autopilot.SetCourseHoldEnabled(false);
  const gnc::ControlContext context{
      .rollDynamics =
          gnc::RollDynamics{
              .aPhi1 = 0.4,
              .aPhi2 = 2.0,
          },
      .rollHoldSettings =
          gnc::RollHoldSettings{
              .targetRollRad = 0.1,
              .dampingRatio = 0.7,
              .naturalFrequencyRadPerSec = 3.0,
          },
  };
  Require(context.rollHoldSettings.has_value(),
      "Course Hold context is missing Roll Hold settings");
  RequireNear(context.rollHoldSettings->naturalFrequencyRadPerSec,
      3.0,
      SimTimeTolerance,
      "Course Hold context did not retain Roll Hold natural frequency");
  Require(
      !courseHold
          ->OnTick(simulation.GetAircraft(), MakeTestTick(simulation), context)
          .has_value(),
      "Disabled Course Hold produced a roll command");

  autopilot.SetCourseHoldEnabled(true);
  Require(autopilot.IsCourseHoldEnabled(),
      "Autopilot did not enable Course Hold");
  RequireNear(autopilot.GetCourseHoldSettings().targetCourseRad,
      settings.targetCourseRad,
      SimTimeTolerance,
      "Course Hold target was not retained");
  RequireNear(autopilot.GetCourseHoldSettings().dampingRatio,
      settings.dampingRatio,
      SimTimeTolerance,
      "Course Hold damping ratio was not retained");
  RequireNear(autopilot.GetCourseHoldSettings().bandwidthSeparationRatio,
      settings.bandwidthSeparationRatio,
      SimTimeTolerance,
      "Course Hold bandwidth separation ratio was not retained");
  const sim::Tick tick = MakeTestTick(simulation);
  const auto &properties = simulation.GetAircraft().GetProperties();
  const double courseErrorRad =
      math::DeltaAngleRad(properties.Course().Rad(), settings.targetCourseRad);
  const double rollNaturalFrequencyRadPerSec =
      context.rollHoldSettings->naturalFrequencyRadPerSec;
  const double courseNaturalFrequencyRadPerSec =
      rollNaturalFrequencyRadPerSec / settings.bandwidthSeparationRatio;
  Require(courseNaturalFrequencyRadPerSec < rollNaturalFrequencyRadPerSec,
      "Course Hold bandwidth was not slower than Roll Hold bandwidth");
  const double gainScale = courseNaturalFrequencyRadPerSec
                           * properties.GroundSpeed().Mps()
                           / properties.GravityMps2();
  const double expectedCommand =
      2.0 * settings.dampingRatio * gainScale * courseErrorRad
      + courseNaturalFrequencyRadPerSec * gainScale
            * (tick.dtSec * courseErrorRad / 2.0);
  const auto command =
      courseHold->OnTick(simulation.GetAircraft(), tick, context);
  Require(command.has_value(), "Enabled Course Hold did not produce a command");
  RequireNear(*command,
      expectedCommand,
      SimTimeTolerance,
      "Course Hold did not apply the Roll/Course bandwidth separation ratio");

  courseHold->Reset();
}

void TestPitchHoldControllerComputesElevatorCommand() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  auto &aircraft = simulation.GetAircraft();
  auto *pitchHold = GetFlightControlManager(simulation)
                        .GetAutopilot()
                        .GetController<gnc::PitchHoldController>();
  Require(pitchHold != nullptr, "Autopilot is missing PitchHoldController");

  const gnc::ControlContext emptyContext{};
  pitchHold->SetEnabled(false);
  Require(!pitchHold->OnTick(aircraft, MakeTestTick(simulation), emptyContext)
              .has_value(),
      "Disabled pitch hold should not produce elevator command");

  const auto &properties = aircraft.GetProperties();
  const gnc::PitchHoldSettings settings{
      .targetPitchRad = properties.Pitch().Rad() + 0.2,
      .dampingRatio = 0.7,
      .naturalFrequencyRadPerSec = 2.0,
  };
  const gnc::ControlContext context{
      .pitchDynamics =
          gnc::PitchDynamics{
              .aTheta1 = 0.4,
              .aTheta2 = 0.8,
              .aTheta3 = -2.0,
          },
  };
  pitchHold->SetTrimElevator(0.1);
  pitchHold->SetSettings(settings);
  pitchHold->SetEnabled(true);

  const auto command =
      pitchHold->OnTick(aircraft, MakeTestTick(simulation), context);
  Require(command.has_value(), "Enabled pitch hold produced no command");

  const gnc::PitchDynamics &dynamics = *context.pitchDynamics;
  const double expectedElevator =
      0.1
      + ComputePitchProportionalGain(settings, dynamics)
            * (settings.targetPitchRad - properties.Pitch().Rad())
      - ComputePitchDerivativeGain(settings, dynamics)
            * properties.Q().RadPerSec();
  RequireNear(*command,
      expectedElevator,
      SimTimeTolerance,
      "Pitch hold elevator command mismatch");
}

void TestAutopilotModeAppliesAutopilotSourceOutput() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  auto &aircraft = simulation.GetAircraft();
  auto &flightControlManager = GetFlightControlManager(simulation);
  auto &manualController = flightControlManager.GetManualController();
  auto &autopilot = flightControlManager.GetAutopilot();
  const gnc::TrimResult *trimResult = autopilot.GetTrimResult();

  Require(trimResult != nullptr, "Autopilot mode test has no stored trim");

  manualController.SetCommandedInput({
      .elevator = 0.2,
      .aileron = -0.8,
      .rudder = 0.1,
      .throttle = 0.4,
  });

  const auto &properties = aircraft.GetProperties();
  const gnc::RollHoldSettings rollSettings{
      .targetRollRad = properties.Roll().Rad() + 0.1,
      .dampingRatio = 0.7,
      .naturalFrequencyRadPerSec = 1.0,
  };
  const gnc::PitchHoldSettings pitchSettings{
      .targetPitchRad = properties.Pitch().Rad() + 0.1,
      .dampingRatio = 0.7,
      .naturalFrequencyRadPerSec = 5.0,
  };
  autopilot.SetRollHoldSettings(rollSettings);
  autopilot.SetRollHoldEnabled(true);
  autopilot.SetPitchHoldSettings(pitchSettings);
  autopilot.SetPitchHoldEnabled(true);
  flightControlManager.SetMode(control::FlightControlMode::Autopilot);

  const auto kickoffStart = std::chrono::steady_clock::now();
  Require(simulation.Tick(), "Asynchronous linearization kickoff tick failed");
  const double kickoffDurationSec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - kickoffStart)
                                        .count();
  Require(kickoffDurationSec < MaximumAsyncKickoffSec,
      "Autopilot linearization blocked the simulation tick");
  Require(!autopilot.GetRollDynamics() && !autopilot.GetPitchDynamics(),
      "Autopilot unexpectedly published dynamics during kickoff");
  const control::ControlInput &kickoffInput = aircraft.GetControls().GetInput();
  RequireNear(kickoffInput.elevator,
      0.2,
      SimTimeTolerance,
      "Autopilot changed elevator before dynamics were ready");
  RequireNear(kickoffInput.aileron,
      -0.8,
      SimTimeTolerance,
      "Autopilot changed aileron before dynamics were ready");

  WaitForAutopilotDynamics(simulation, autopilot, true, true);
  const auto rollDynamics = autopilot.GetRollDynamics();
  const auto pitchDynamics = autopilot.GetPitchDynamics();
  Require(rollDynamics.has_value(), "Autopilot did not provide roll dynamics");
  Require(pitchDynamics.has_value(),
      "Autopilot did not provide pitch dynamics");
  const gnc::LinearizationResult *linearization =
      autopilot.GetLinearizationResult();
  Require(linearization != nullptr,
      "Autopilot did not publish the periodic linearization result");
  Require(
      linearization->A.rows()
              == static_cast<Eigen::Index>(linearization->stateNames.size())
          && linearization->A.cols()
                 == static_cast<Eigen::Index>(linearization->stateNames.size()),
      "Periodic system matrix dimensions do not match state names");
  Require(
      linearization->B.rows()
              == static_cast<Eigen::Index>(linearization->stateNames.size())
          && linearization->B.cols()
                 == static_cast<Eigen::Index>(linearization->inputNames.size()),
      "Periodic input matrix dimensions do not match state and input names");

  const double rollBeforeTick = properties.Roll().Rad();
  const double rollRateBeforeTick = properties.P().RadPerSec();
  const double pitchBeforeTick = properties.Pitch().Rad();
  const double pitchRateBeforeTick = properties.Q().RadPerSec();

  Require(simulation.Tick(), "Autopilot mode tick failed");

  const double expectedElevator =
      control::ClampControlAxisValue(control::ControlAxis::Elevator,
          trimResult->elevator
              + ComputePitchProportionalGain(pitchSettings, *pitchDynamics)
                    * (pitchSettings.targetPitchRad - pitchBeforeTick)
              - ComputePitchDerivativeGain(pitchSettings, *pitchDynamics)
                    * pitchRateBeforeTick);
  const double expectedAileron =
      control::ClampControlAxisValue(control::ControlAxis::Aileron,
          trimResult->aileron
              + ComputeRollProportionalGain(rollSettings, *rollDynamics)
                    * (rollSettings.targetRollRad - rollBeforeTick)
              - ComputeRollDerivativeGain(rollSettings, *rollDynamics)
                    * rollRateBeforeTick);
  const control::ControlInput &actualInput = aircraft.GetControls().GetInput();
  RequireNear(actualInput.elevator,
      expectedElevator,
      ControlCommandTolerance,
      "Autopilot mode did not apply pitch hold elevator");
  RequireNear(actualInput.aileron,
      expectedAileron,
      ControlCommandTolerance,
      "Autopilot mode did not apply roll hold aileron");
  RequireNear(actualInput.rudder,
      0.1,
      SimTimeTolerance,
      "Autopilot mode should pass through manual rudder");
  RequireNear(actualInput.throttle,
      0.4,
      SimTimeTolerance,
      "Autopilot mode should pass through manual throttle");
}

void TestPitchHoldOnlyPassesThroughManualLateralAxes() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  auto &aircraft = simulation.GetAircraft();
  auto &flightControlManager = GetFlightControlManager(simulation);
  auto &manualController = flightControlManager.GetManualController();
  auto &autopilot = flightControlManager.GetAutopilot();
  const gnc::TrimResult *trimResult = autopilot.GetTrimResult();

  Require(trimResult != nullptr, "Pitch hold pass-through test has no trim");

  manualController.SetCommandedInput({
      .elevator = 0.2,
      .aileron = -0.8,
      .rudder = 0.1,
      .throttle = 0.4,
  });

  const auto &properties = aircraft.GetProperties();
  const gnc::PitchHoldSettings pitchSettings{
      .targetPitchRad = properties.Pitch().Rad() + 0.1,
      .dampingRatio = 0.7,
      .naturalFrequencyRadPerSec = 5.0,
  };
  autopilot.SetPitchHoldSettings(pitchSettings);
  autopilot.SetPitchHoldEnabled(true);
  autopilot.SetRollHoldEnabled(false);
  flightControlManager.SetMode(control::FlightControlMode::Autopilot);

  WaitForAutopilotDynamics(simulation, autopilot, false, true);
  const auto pitchDynamics = autopilot.GetPitchDynamics();
  Require(pitchDynamics.has_value(),
      "Autopilot did not provide pitch dynamics");

  const double pitchBeforeTick = properties.Pitch().Rad();
  const double pitchRateBeforeTick = properties.Q().RadPerSec();

  Require(simulation.Tick(), "Pitch hold pass-through tick failed");

  const double expectedElevator =
      control::ClampControlAxisValue(control::ControlAxis::Elevator,
          trimResult->elevator
              + ComputePitchProportionalGain(pitchSettings, *pitchDynamics)
                    * (pitchSettings.targetPitchRad - pitchBeforeTick)
              - ComputePitchDerivativeGain(pitchSettings, *pitchDynamics)
                    * pitchRateBeforeTick);
  const control::ControlInput &actualInput = aircraft.GetControls().GetInput();
  RequireNear(actualInput.elevator,
      expectedElevator,
      ControlCommandTolerance,
      "Pitch hold did not apply elevator");
  RequireNear(actualInput.aileron,
      -0.8,
      SimTimeTolerance,
      "Pitch hold should pass through manual aileron");
  RequireNear(actualInput.rudder,
      0.1,
      SimTimeTolerance,
      "Pitch hold should pass through manual rudder");
  RequireNear(actualInput.throttle,
      0.4,
      SimTimeTolerance,
      "Pitch hold should pass through manual throttle");
}

void TestRollHoldOnlyPassesThroughManualLongitudinalAxes() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  auto &aircraft = simulation.GetAircraft();
  auto &flightControlManager = GetFlightControlManager(simulation);
  auto &manualController = flightControlManager.GetManualController();
  auto &autopilot = flightControlManager.GetAutopilot();
  const gnc::TrimResult *trimResult = autopilot.GetTrimResult();

  Require(trimResult != nullptr, "Roll hold pass-through test has no trim");

  manualController.SetCommandedInput({
      .elevator = 0.2,
      .aileron = -0.8,
      .rudder = 0.1,
      .throttle = 0.4,
  });

  const auto &properties = aircraft.GetProperties();
  const gnc::RollHoldSettings rollSettings{
      .targetRollRad = properties.Roll().Rad() + 0.1,
      .dampingRatio = 0.7,
      .naturalFrequencyRadPerSec = 1.0,
  };
  autopilot.SetRollHoldSettings(rollSettings);
  autopilot.SetRollHoldEnabled(true);
  autopilot.SetPitchHoldEnabled(false);
  flightControlManager.SetMode(control::FlightControlMode::Autopilot);

  WaitForAutopilotDynamics(simulation, autopilot, true, false);
  const auto rollDynamics = autopilot.GetRollDynamics();
  Require(rollDynamics.has_value(), "Autopilot did not provide roll dynamics");

  const double rollBeforeTick = properties.Roll().Rad();
  const double rollRateBeforeTick = properties.P().RadPerSec();

  Require(simulation.Tick(), "Roll hold pass-through tick failed");

  const double expectedAileron =
      control::ClampControlAxisValue(control::ControlAxis::Aileron,
          trimResult->aileron
              + ComputeRollProportionalGain(rollSettings, *rollDynamics)
                    * (rollSettings.targetRollRad - rollBeforeTick)
              - ComputeRollDerivativeGain(rollSettings, *rollDynamics)
                    * rollRateBeforeTick);
  const control::ControlInput &actualInput = aircraft.GetControls().GetInput();
  RequireNear(actualInput.elevator,
      0.2,
      SimTimeTolerance,
      "Roll hold should pass through manual elevator");
  RequireNear(actualInput.aileron,
      expectedAileron,
      ControlCommandTolerance,
      "Roll hold did not apply aileron");
  RequireNear(actualInput.rudder,
      0.1,
      SimTimeTolerance,
      "Roll hold should pass through manual rudder");
  RequireNear(actualInput.throttle,
      0.4,
      SimTimeTolerance,
      "Roll hold should pass through manual throttle");
}

void TestFDMStateFlagOperations() {
  sim::FDMStateFlags flags = sim::FDMStateFlags::None;
  flags |= sim::FDMStateFlags::State;
  flags |= sim::FDMStateFlags::Controls;

  Require(sim::HasFDMStateFlag(flags, sim::FDMStateFlags::State),
      "State flag was not set");
  Require(sim::HasFDMStateFlag(flags, sim::FDMStateFlags::Controls),
      "Controls flag was not set");
  Require(!sim::HasFDMStateFlag(flags, sim::FDMStateFlags::Propulsion),
      "Propulsion flag was unexpectedly set");

  flags ^= sim::FDMStateFlags::Controls;
  Require(!sim::HasFDMStateFlag(flags, sim::FDMStateFlags::Controls),
      "XOR did not clear Controls flag");

  flags = sim::FDMStateFlags::All & ~sim::FDMStateFlags::Environment;
  Require(!sim::HasFDMStateFlag(flags, sim::FDMStateFlags::Environment),
      "Complement did not exclude Environment flag");
  Require(sim::HasFDMStateFlag(flags, sim::FDMStateFlags::Propulsion),
      "Complement removed an unrelated flag");
}

void TestFDMStateAndControlSynchronization() {
  sim::Aircraft source;
  sim::Aircraft target;

  sim::InitialCondition sourceCondition{};
  sourceCondition.latitudeDeg = 37.45;
  sourceCondition.longitudeDeg = 127.11;
  sourceCondition.altitudeFt = 3500.0;
  sourceCondition.rollDeg = 8.0;
  sourceCondition.pitchDeg = -3.0;
  sourceCondition.headingDeg = 42.0;
  sourceCondition.airspeedKts = 105.0;
  sourceCondition.pRadPerSec = 0.03;
  sourceCondition.qRadPerSec = -0.02;
  sourceCondition.rRadPerSec = 0.01;

  sim::InitialCondition targetCondition{};
  targetCondition.latitudeDeg = -12.0;
  targetCondition.longitudeDeg = 15.0;
  targetCondition.altitudeFt = 900.0;
  targetCondition.headingDeg = 210.0;
  targetCondition.airspeedKts = 70.0;

  Require(source.Initialize(MakeConfig(), sourceCondition),
      "Source Aircraft failed to initialize");
  Require(target.Initialize(MakeConfig(), targetCondition),
      "Target Aircraft failed to initialize");

  sim::FDMState sourceControlSetup =
      source.ExtractFDMState(sim::FDMStateFlags::Controls);
  sourceControlSetup.controls.elevatorCommand = 0.21;
  sourceControlSetup.controls.aileronCommand = -0.32;
  sourceControlSetup.controls.rudderCommand = 0.17;
  sourceControlSetup.controls.pitchTrimCommand = -0.08;
  std::fill(sourceControlSetup.controls.throttleCommands.begin(),
      sourceControlSetup.controls.throttleCommands.end(),
      0.64);
  sourceControlSetup.controls.elevatorPositionRad = 0.07;
  sourceControlSetup.controls.leftAileronPositionRad = -0.05;
  sourceControlSetup.controls.rightAileronPositionRad = 0.05;
  sourceControlSetup.controls.rudderPositionRad = 0.04;
  std::fill(sourceControlSetup.controls.throttlePositions.begin(),
      sourceControlSetup.controls.throttlePositions.end(),
      0.61);
  source.ApplyFDMState(sourceControlSetup);

  sim::FDMState targetControlSetup =
      target.ExtractFDMState(sim::FDMStateFlags::Controls);
  targetControlSetup.controls.elevatorCommand = -0.45;
  targetControlSetup.controls.aileronCommand = 0.36;
  targetControlSetup.controls.rudderCommand = -0.27;
  targetControlSetup.controls.pitchTrimCommand = 0.12;
  std::fill(targetControlSetup.controls.throttleCommands.begin(),
      targetControlSetup.controls.throttleCommands.end(),
      0.22);
  target.ApplyFDMState(targetControlSetup);

  const sim::FDMState targetControlsBeforeStateApply =
      target.ExtractFDMState(sim::FDMStateFlags::Controls);
  const double targetTimeBeforeStateApply =
      target.GetAircraftState().simulationTimeSec;
  const sim::FDMState sourceState =
      source.ExtractFDMState(sim::FDMStateFlags::State);

  Require(sourceState.flags == sim::FDMStateFlags::State,
      "Extracted state did not preserve requested flags");
  Require(sourceState.controls.throttleCommands.empty(),
      "State-only extraction populated Controls data");

  target.ApplyFDMState(sourceState);
  const sim::FDMState synchronizedState =
      target.ExtractFDMState(sim::FDMStateFlags::State);
  const sim::FDMState targetControlsAfterStateApply =
      target.ExtractFDMState(sim::FDMStateFlags::Controls);

  RequireKinematicStateNear(synchronizedState.state,
      sourceState.state,
      "State synchronization");
  RequireControlStateNear(targetControlsAfterStateApply.controls,
      targetControlsBeforeStateApply.controls,
      "State-only synchronization changed Controls");
  RequireNear(target.GetAircraftState().simulationTimeSec,
      targetTimeBeforeStateApply,
      SimTimeTolerance,
      "State synchronization advanced simulation time");

  const sim::FDMState targetStateBeforeControlApply =
      target.ExtractFDMState(sim::FDMStateFlags::State);
  const sim::FDMState sourceControls =
      source.ExtractFDMState(sim::FDMStateFlags::Controls);
  target.ApplyFDMState(sourceControls);
  const sim::FDMState synchronizedControls =
      target.ExtractFDMState(sim::FDMStateFlags::Controls);
  const sim::FDMState targetStateAfterControlApply =
      target.ExtractFDMState(sim::FDMStateFlags::State);

  RequireControlStateNear(synchronizedControls.controls,
      sourceControls.controls,
      "Control synchronization");
  RequireKinematicStateNear(targetStateAfterControlApply.state,
      targetStateBeforeControlApply.state,
      "Controls-only synchronization changed State");
  RequireNear(target.GetAircraftState().simulationTimeSec,
      targetTimeBeforeStateApply,
      SimTimeTolerance,
      "Control synchronization advanced simulation time");
}

void TestFDMPropulsionAndEnvironmentSynchronization() {
  sim::Aircraft source;
  sim::Aircraft target;
  Require(source.Initialize(MakeConfig(), {}),
      "Source Aircraft failed to initialize");
  Require(target.Initialize(MakeConfig(), {}),
      "Target Aircraft failed to initialize");

  constexpr sim::FDMStateFlags Flags =
      sim::FDMStateFlags::Propulsion | sim::FDMStateFlags::Environment;
  sim::FDMState sourceSetup = source.ExtractFDMState(Flags);
  Require(!sourceSetup.propulsion.engines.empty(),
      "Expected at least one engine for propulsion synchronization");

  sourceSetup.propulsion.engines[0].running = true;
  sourceSetup.propulsion.engines[0].engineRpm = 1350.0;
  sourceSetup.propulsion.engines[0].thrusterRpm = 1350.0;
  sourceSetup.environment.temperatureBiasRankine = 7.0;
  sourceSetup.environment.seaLevelGradedTemperatureDeltaRankine = 3.0;
  sourceSetup.environment.vaporMassFractionPpm = 2500.0;
  sourceSetup.environment.seaLevelPressurePsf = 2075.0;
  sourceSetup.environment.windNedFps = {12.0, -7.0, 2.0};
  sourceSetup.environment.gustNedFps = {1.5, -0.5, 0.25};
  sourceSetup.environment.turbulenceNedFps = {0.3, 0.2, -0.1};
  sourceSetup.environment.turbulenceGain = 0.4;
  sourceSetup.environment.turbulenceRate = 0.8;
  sourceSetup.environment.turbulenceRhythmicity = 0.6;
  sourceSetup.environment.windSpeedAt20FtFps = 9.0;
  sourceSetup.environment.terrainElevationFt = 350.0;

  source.ApplyFDMState(sourceSetup);
  const sim::FDMState sourceState = source.ExtractFDMState(Flags);
  const double targetTimeBeforeApply =
      target.GetAircraftState().simulationTimeSec;
  target.ApplyFDMState(sourceState);
  const sim::FDMState targetState = target.ExtractFDMState(Flags);

  Require(targetState.propulsion.engines.size()
              == sourceState.propulsion.engines.size(),
      "Synchronized engine count mismatch");
  for (std::size_t index = 0; index < sourceState.propulsion.engines.size();
      ++index) {
    const sim::FDMEngineState &actual = targetState.propulsion.engines[index];
    const sim::FDMEngineState &expected = sourceState.propulsion.engines[index];
    Require(actual.running == expected.running,
        "Synchronized engine running state mismatch");
    RequireNear(actual.engineRpm,
        expected.engineRpm,
        SimTimeTolerance,
        "Synchronized engine RPM mismatch");
    RequireNear(actual.thrusterRpm,
        expected.thrusterRpm,
        SimTimeTolerance,
        "Synchronized thruster RPM mismatch");
  }

  const sim::FDMEnvironmentState &actual = targetState.environment;
  const sim::FDMEnvironmentState &expected = sourceState.environment;
  RequireNear(actual.seaLevelTemperatureRankine,
      expected.seaLevelTemperatureRankine,
      SimTimeTolerance,
      "Synchronized sea-level temperature mismatch");
  RequireNear(actual.seaLevelPressurePsf,
      expected.seaLevelPressurePsf,
      SimTimeTolerance,
      "Synchronized sea-level pressure mismatch");
  Require(actual.hasStandardAtmosphere == expected.hasStandardAtmosphere,
      "Synchronized atmosphere model mismatch");
  RequireNear(actual.temperatureBiasRankine,
      expected.temperatureBiasRankine,
      SimTimeTolerance,
      "Synchronized temperature bias mismatch");
  RequireNear(actual.seaLevelGradedTemperatureDeltaRankine,
      expected.seaLevelGradedTemperatureDeltaRankine,
      SimTimeTolerance,
      "Synchronized graded temperature delta mismatch");
  RequireNear(actual.vaporMassFractionPpm,
      expected.vaporMassFractionPpm,
      SimTimeTolerance,
      "Synchronized vapor fraction mismatch");
  RequireArrayNear(actual.windNedFps,
      expected.windNedFps,
      SimTimeTolerance,
      "Synchronized wind mismatch");
  RequireArrayNear(actual.gustNedFps,
      expected.gustNedFps,
      SimTimeTolerance,
      "Synchronized gust mismatch");
  RequireArrayNear(actual.turbulenceNedFps,
      expected.turbulenceNedFps,
      SimTimeTolerance,
      "Synchronized turbulence mismatch");
  Require(actual.turbulenceType == expected.turbulenceType,
      "Synchronized turbulence type mismatch");
  RequireNear(actual.turbulenceGain,
      expected.turbulenceGain,
      SimTimeTolerance,
      "Synchronized turbulence gain mismatch");
  RequireNear(actual.turbulenceRate,
      expected.turbulenceRate,
      SimTimeTolerance,
      "Synchronized turbulence rate mismatch");
  RequireNear(actual.turbulenceRhythmicity,
      expected.turbulenceRhythmicity,
      SimTimeTolerance,
      "Synchronized turbulence rhythmicity mismatch");
  RequireNear(actual.windSpeedAt20FtFps,
      expected.windSpeedAt20FtFps,
      SimTimeTolerance,
      "Synchronized 20-foot wind speed mismatch");
  RequireNear(actual.terrainElevationFt,
      expected.terrainElevationFt,
      1.0e-6,
      "Synchronized terrain elevation mismatch");
  Require(actual.gravityType == expected.gravityType,
      "Synchronized gravity type mismatch");
  RequireNear(actual.planetRotationRateRadPerSec,
      expected.planetRotationRateRadPerSec,
      SimTimeTolerance,
      "Synchronized planet rotation rate mismatch");
  RequireNear(target.GetAircraftState().simulationTimeSec,
      targetTimeBeforeApply,
      SimTimeTolerance,
      "Propulsion/environment synchronization advanced simulation time");
}

} // namespace

int main() {
  try {
    TestErrorTrackerOwnsErrorState();
    TestSimulationComponentLifecycle();
    TestTickAdvancesOneStep();
    TestResetUsesDefaultInitialCondition();
    TestResetWithInitialCondition();
    TestCaptureCurrentStateCanReset();
    TestEngineStateInspection();
    TestInvalidInitialConditionFails();
    TestAircraftStateAccess();
    TestNavigationProperties();
    TestStartAppliesInitialTrim();
    TestInitialTrimIsStoredInAutopilot();
    TestAutopilotControllerRegistry();
    TestControlSystemAxisSettersClampFinalInput();
    TestControlSystemSetInputClampsFinalInput();
    TestManualFlightControlControllerAppliesCommands();
    TestFlightControlManagerOwnsAndRoutesControllers();
    TestFlightControlManagerNoInputPreservesCommand();
    TestManualModeIgnoresAutopilotSource();
    TestLinearizationRunsInManualModeWithoutHolds();
    TestRollHoldControllerComputesAileronCommand();
    TestCourseHoldCommandInterface();
    TestPitchHoldControllerComputesElevatorCommand();
    TestAutopilotModeAppliesAutopilotSourceOutput();
    TestPitchHoldOnlyPassesThroughManualLateralAxes();
    TestRollHoldOnlyPassesThroughManualLongitudinalAxes();
    TestFDMStateFlagOperations();
    TestFDMStateAndControlSynchronization();
    TestFDMPropulsionAndEnvironmentSynchronization();
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

  return 0;
}
