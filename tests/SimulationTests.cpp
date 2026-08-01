#include "application/sim/Aircraft.hpp"
#include "application/sim/ErrorTracker.hpp"
#include "application/sim/Simulation.hpp"
#include "application/sim/StateLogger.hpp"
#include "application/sim/control/FlightControlManager.hpp"
#include "application/sim/control/FlightControlMode.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
constexpr double SimTimeTolerance = 1.0e-9;
constexpr double AltitudeToleranceFt = 1.0;
constexpr double AirspeedToleranceKts = 0.5;
constexpr double HeadingToleranceDeg = 0.5;
constexpr double TrimInputTolerance = 1.0e-5;
constexpr double DegToRad = 0.017453292519943295769;

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
      aircraftState.rollDeg * DegToRad,
      SimTimeTolerance,
      "Flight properties roll rad mismatch");
  RequireNear(properties.Pitch().Rad(),
      aircraftState.pitchDeg * DegToRad,
      SimTimeTolerance,
      "Flight properties pitch rad mismatch");
  RequireNear(properties.P().RadPerSec(),
      aircraftState.pDegPerSec * DegToRad,
      SimTimeTolerance,
      "Flight properties roll rate rad mismatch");
  RequireNear(properties.P().DotRadPerSec2(),
      derivative.pDotDegPerSec2 * DegToRad,
      SimTimeTolerance,
      "Flight properties roll acceleration rad mismatch");
  RequireNear(properties.TrueAirspeed().Mps(),
      aircraftState.trueAirspeedMps,
      SimTimeTolerance,
      "Flight properties true airspeed mps mismatch");
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
      .proportionalGain = 0.5,
      .derivativeGain = 0.0,
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

void TestRollHoldControllerComputesAileronCommand() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  auto &aircraft = simulation.GetAircraft();
  auto *rollHold = GetFlightControlManager(simulation)
                       .GetAutopilot()
                       .GetController<gnc::RollHoldController>();
  Require(rollHold != nullptr, "Autopilot is missing RollHoldController");

  rollHold->SetEnabled(false);
  Require(!rollHold->OnTick(aircraft, MakeTestTick(simulation)).has_value(),
      "Disabled roll hold should not produce aileron command");

  const auto &properties = aircraft.GetProperties();
  const double targetRollRad = properties.Roll().Rad() + 0.2;
  rollHold->SetTrimAileron(0.1);
  rollHold->SetSettings({
      .targetRollRad = targetRollRad,
      .proportionalGain = 0.5,
      .derivativeGain = 0.25,
  });
  rollHold->SetEnabled(true);

  const auto command = rollHold->OnTick(aircraft, MakeTestTick(simulation));
  Require(command.has_value(), "Enabled roll hold produced no command");

  const double expectedAileron =
      0.1 + 0.5 * (targetRollRad - properties.Roll().Rad())
      - 0.25 * properties.P().RadPerSec();
  RequireNear(*command,
      expectedAileron,
      SimTimeTolerance,
      "Roll hold aileron command mismatch");
}

void TestPitchHoldControllerComputesElevatorCommand() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  auto &aircraft = simulation.GetAircraft();
  auto *pitchHold = GetFlightControlManager(simulation)
                        .GetAutopilot()
                        .GetController<gnc::PitchHoldController>();
  Require(pitchHold != nullptr, "Autopilot is missing PitchHoldController");

  pitchHold->SetEnabled(false);
  Require(!pitchHold->OnTick(aircraft, MakeTestTick(simulation)).has_value(),
      "Disabled pitch hold should not produce elevator command");

  const auto &properties = aircraft.GetProperties();
  const double targetPitchRad = properties.Pitch().Rad() + 0.2;
  pitchHold->SetTrimElevator(0.1);
  pitchHold->SetSettings({
      .targetPitchRad = targetPitchRad,
      .proportionalGain = 0.5,
      .derivativeGain = 0.25,
  });
  pitchHold->SetEnabled(true);

  const auto command = pitchHold->OnTick(aircraft, MakeTestTick(simulation));
  Require(command.has_value(), "Enabled pitch hold produced no command");

  const double expectedElevator =
      0.1 - 0.5 * (targetPitchRad - properties.Pitch().Rad())
      + 0.25 * properties.Q().RadPerSec();
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
  const double rollTargetRad = properties.Roll().Rad() + 0.1;
  const double pitchTargetRad = properties.Pitch().Rad() + 0.1;
  autopilot.SetRollHoldSettings({
      .targetRollRad = rollTargetRad,
      .proportionalGain = 0.5,
      .derivativeGain = 0.0,
  });
  autopilot.SetRollHoldEnabled(true);
  autopilot.SetPitchHoldSettings({
      .targetPitchRad = pitchTargetRad,
      .proportionalGain = 0.5,
      .derivativeGain = 0.0,
  });
  autopilot.SetPitchHoldEnabled(true);
  flightControlManager.SetMode(control::FlightControlMode::Autopilot);

  const double expectedElevator = control::ClampControlAxisValue(
      control::ControlAxis::Elevator,
      trimResult->elevator - 0.5 * (pitchTargetRad - properties.Pitch().Rad()));
  const double expectedAileron = control::ClampControlAxisValue(
      control::ControlAxis::Aileron,
      trimResult->aileron + 0.5 * (rollTargetRad - properties.Roll().Rad()));

  Require(simulation.Tick(), "Autopilot mode tick failed");

  const control::ControlInput &actualInput = aircraft.GetControls().GetInput();
  RequireNear(actualInput.elevator,
      expectedElevator,
      SimTimeTolerance,
      "Autopilot mode did not apply pitch hold elevator");
  RequireNear(actualInput.aileron,
      expectedAileron,
      SimTimeTolerance,
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
  const double pitchTargetRad = properties.Pitch().Rad() + 0.1;
  autopilot.SetPitchHoldSettings({
      .targetPitchRad = pitchTargetRad,
      .proportionalGain = 0.5,
      .derivativeGain = 0.0,
  });
  autopilot.SetPitchHoldEnabled(true);
  autopilot.SetRollHoldEnabled(false);
  flightControlManager.SetMode(control::FlightControlMode::Autopilot);

  const double expectedElevator = control::ClampControlAxisValue(
      control::ControlAxis::Elevator,
      trimResult->elevator - 0.5 * (pitchTargetRad - properties.Pitch().Rad()));

  Require(simulation.Tick(), "Pitch hold pass-through tick failed");

  const control::ControlInput &actualInput = aircraft.GetControls().GetInput();
  RequireNear(actualInput.elevator,
      expectedElevator,
      SimTimeTolerance,
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
  const double rollTargetRad = properties.Roll().Rad() + 0.1;
  autopilot.SetRollHoldSettings({
      .targetRollRad = rollTargetRad,
      .proportionalGain = 0.5,
      .derivativeGain = 0.0,
  });
  autopilot.SetRollHoldEnabled(true);
  autopilot.SetPitchHoldEnabled(false);
  flightControlManager.SetMode(control::FlightControlMode::Autopilot);

  const double expectedAileron = control::ClampControlAxisValue(
      control::ControlAxis::Aileron,
      trimResult->aileron + 0.5 * (rollTargetRad - properties.Roll().Rad()));

  Require(simulation.Tick(), "Roll hold pass-through tick failed");

  const control::ControlInput &actualInput = aircraft.GetControls().GetInput();
  RequireNear(actualInput.elevator,
      0.2,
      SimTimeTolerance,
      "Roll hold should pass through manual elevator");
  RequireNear(actualInput.aileron,
      expectedAileron,
      SimTimeTolerance,
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
    TestStartAppliesInitialTrim();
    TestInitialTrimIsStoredInAutopilot();
    TestAutopilotControllerRegistry();
    TestControlSystemAxisSettersClampFinalInput();
    TestControlSystemSetInputClampsFinalInput();
    TestManualFlightControlControllerAppliesCommands();
    TestFlightControlManagerOwnsAndRoutesControllers();
    TestFlightControlManagerNoInputPreservesCommand();
    TestManualModeIgnoresAutopilotSource();
    TestRollHoldControllerComputesAileronCommand();
    TestPitchHoldControllerComputesElevatorCommand();
    TestAutopilotModeAppliesAutopilotSourceOutput();
    TestPitchHoldOnlyPassesThroughManualLateralAxes();
    TestRollHoldOnlyPassesThroughManualLongitudinalAxes();
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

  return 0;
}
