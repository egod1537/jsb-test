#include "application/sim/Aircraft.hpp"
#include "application/sim/Simulation.hpp"
#include "application/sim/control/FlightControlMode.hpp"
#include "application/sim/state/IStateProvider.hpp"

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
  config.altitudeFt = 1000.0;
  config.calibratedAirspeedKts = 80.0;
  config.headingDeg = 0.0;
  return config;
}

void StartSimulation(sim::Simulation &simulation) {
  Require(simulation.Start(MakeConfig()), "Simulation failed to start");
}

double GetSimTime(const sim::Simulation &simulation) {
  return simulation.GetAircraft().GetAircraftState().simulationTimeSec;
}

void TestPauseStopsTime() {
  sim::Simulation simulation;
  StartSimulation(simulation);

  Require(simulation.Update(), "Initial update failed");
  simulation.Pause();
  const double pausedTime = GetSimTime(simulation);

  for (int i = 0; i < 5; ++i) {
    Require(simulation.Update(), "Paused update failed");
  }

  RequireNear(GetSimTime(simulation),
      pausedTime,
      SimTimeTolerance,
      "Paused simulation time advanced");
}

void TestStepOnceAdvancesOneTick() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  simulation.Pause();

  const double startTime = GetSimTime(simulation);
  Require(simulation.RequestStep(), "Step request while paused failed");
  Require(simulation.Update(), "Stepped update failed");

  const double steppedTime = GetSimTime(simulation);
  RequireNear(steppedTime,
      startTime + simulation.GetTickSizeSec(),
      SimTimeTolerance,
      "Step once did not advance by one tick");

  for (int i = 0; i < 3; ++i) {
    Require(simulation.Update(), "Post-step paused update failed");
  }

  RequireNear(GetSimTime(simulation),
      steppedTime,
      SimTimeTolerance,
      "Paused simulation advanced after consuming step request");
}

void TestResumeAdvancesTime() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  simulation.Pause();
  const double pausedTime = GetSimTime(simulation);

  simulation.Resume();
  Require(simulation.Update(), "Resume update failed");

  Require(GetSimTime(simulation) > pausedTime,
      "Simulation time did not advance after resume");
}

void TestRestartUsesStoredInitialCondition() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  for (int i = 0; i < 3; ++i) {
    Require(simulation.Update(), "Pre-restart update failed");
  }

  Require(simulation.Restart(), "Restart failed");

  RequireNear(GetSimTime(simulation),
      0.0,
      SimTimeTolerance,
      "Restart did not reset simulation time");

  const sim::InitialCondition captured = simulation.CaptureCurrentCondition();
  RequireNear(captured.altitudeFt,
      simulation.GetInitialCondition().altitudeFt,
      AltitudeToleranceFt,
      "Restart altitude does not match stored IC");
  RequireNear(captured.airspeedKts,
      simulation.GetInitialCondition().airspeedKts,
      AirspeedToleranceKts,
      "Restart airspeed does not match stored IC");
}

void TestRestartWithInitialCondition() {
  sim::Simulation simulation;
  StartSimulation(simulation);

  sim::InitialCondition initialCondition = simulation.GetInitialCondition();
  initialCondition.altitudeFt = 2500.0;
  initialCondition.headingDeg = 45.0;
  initialCondition.airspeedKts = 95.0;

  Require(simulation.Restart(initialCondition),
      "Restart with custom IC failed");

  const sim::InitialCondition captured = simulation.CaptureCurrentCondition();
  RequireNear(captured.altitudeFt,
      initialCondition.altitudeFt,
      AltitudeToleranceFt,
      "Custom restart altitude mismatch");
  RequireNear(captured.headingDeg,
      initialCondition.headingDeg,
      HeadingToleranceDeg,
      "Custom restart heading mismatch");
  RequireNear(captured.airspeedKts,
      initialCondition.airspeedKts,
      AirspeedToleranceKts,
      "Custom restart airspeed mismatch");
}

void TestCaptureCurrentStateCanRestart() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  Require(simulation.Update(), "Update before capture failed");

  const sim::InitialCondition captured = simulation.CaptureCurrentCondition();
  Require(simulation.Restart(captured), "Restart with captured state failed");
  const sim::InitialCondition restored = simulation.CaptureCurrentCondition();

  RequireNear(restored.altitudeFt,
      captured.altitudeFt,
      AltitudeToleranceFt,
      "Captured restart altitude mismatch");
  RequireNear(restored.headingDeg,
      captured.headingDeg,
      HeadingToleranceDeg,
      "Captured restart heading mismatch");
  RequireNear(restored.airspeedKts,
      captured.airspeedKts,
      AirspeedToleranceKts,
      "Captured restart airspeed mismatch");
}

void TestEngineStateInspection() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  const auto &aircraft = simulation.GetAircraft();
  const std::size_t engineCount = aircraft.GetEngineCount();

  Require(engineCount >= 1, "Expected at least one engine");

  const sim::EngineState engineState = aircraft.GetEngineState(0);

  Require(engineState.index == 0, "Engine index mismatch");
  if (engineCount == 1) {
    Require(engineState.running == aircraft.IsAnyEngineRunning(),
        "Single engine running state differs from aggregate query");
    Require(engineState.running == aircraft.AreAllEnginesRunning(),
        "Single engine running state differs from all-engines query");
  }
  Require(std::isfinite(engineState.rpm), "Engine RPM is not finite");
  Require(std::isfinite(engineState.throttleCommand),
      "Engine throttle command is not finite");

  const sim::EngineState invalidEngineState =
      aircraft.GetEngineState(engineCount + 1);
  Require(invalidEngineState.index == engineCount + 1,
      "Invalid engine index was not preserved");
}

void TestInvalidInitialConditionFails() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  sim::InitialCondition invalid = simulation.GetInitialCondition();
  invalid.latitudeDeg = 100.0;

  Require(!simulation.SetInitialCondition(invalid),
      "Invalid latitude was accepted");
  Require(simulation.GetLastError().has_value(),
      "Invalid IC did not report an error");
}

void TestStateProvider() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  const state::AircraftState state = simulation.GetStateProvider().GetState();
  const sim::AircraftState aircraftState =
      simulation.GetAircraft().GetAircraftState();
  const sim::AircraftStateDerivative derivative =
      simulation.GetAircraft().GetAircraftStateDerivative();
  const auto &properties = simulation.GetAircraft().GetProperties();

  RequireNear(state.simTimeSec,
      aircraftState.simulationTimeSec,
      SimTimeTolerance,
      "State provider simulation time mismatch");
  Require(std::isfinite(state.altitudeM), "State provider altitude invalid");
  Require(std::isfinite(state.airspeedMps), "State provider airspeed invalid");
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
  const control::ControlInput &input = aircraft.GetAircraftControlInput();
  const double pitchTrim = aircraft.GetFlightControls().GetPitchTrim();

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
  const auto &autopilot = simulation.GetAutopilot();
  const gnc::TrimResult *trimResult = autopilot.GetTrimResult();

  Require(trimResult != nullptr, "Autopilot did not store initial trim result");
  Require(trimResult->success, "Autopilot stored a failed initial trim result");
}

void TestAircraftAxisSettersClampFinalInput() {
  sim::Aircraft aircraft;

  Require(aircraft.SetElevatorInput(-2.0), "Elevator setter did not change");
  Require(aircraft.SetAileronInput(2.0), "Aileron setter did not change");
  Require(aircraft.SetRudderInput(3.0), "Rudder setter did not change");
  Require(aircraft.SetThrottleInput(0.5), "Throttle setter did not change");
  Require(aircraft.SetThrottleInput(-1.0), "Throttle setter did not change");

  const control::ControlInput &input = aircraft.GetAircraftControlInput();
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

void TestAircraftSetAircraftControlInputClampsFinalInput() {
  sim::Aircraft aircraft;

  aircraft.SetAircraftControlInput({
      .elevator = -2.0,
      .aileron = 2.0,
      .rudder = 3.0,
      .throttle = 2.0,
  });

  const control::ControlInput &input = aircraft.GetAircraftControlInput();
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
  auto &manualController = simulation.GetManualFlightControlController();

  simulation.SetFlightControlMode(control::FlightControlMode::Manual);
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

  Require(simulation.Update(), "Manual flight control update failed");

  const control::ControlInput &actualInput =
      aircraft.GetAircraftControlInput();
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

void TestManualModeIgnoresAutopilotController() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  auto &aircraft = simulation.GetAircraft();
  auto &manualController = simulation.GetManualFlightControlController();
  auto &autopilot = simulation.GetAutopilot();

  simulation.SetFlightControlMode(control::FlightControlMode::Manual);
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

  Require(simulation.Update(), "Manual mode update failed");

  const control::ControlInput &actualInput = aircraft.GetAircraftControlInput();
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
  auto &rollHold = simulation.GetAutopilot().GetRollHoldController();

  rollHold.SetEnabled(false);
  Require(!rollHold.Update(aircraft, simulation.GetTickSizeSec()).has_value(),
      "Disabled roll hold should not produce aileron command");

  const auto &properties = aircraft.GetProperties();
  const double targetRollRad = properties.Roll().Rad() + 0.2;
  rollHold.SetTrimAileron(0.1);
  rollHold.SetSettings({
      .targetRollRad = targetRollRad,
      .proportionalGain = 0.5,
      .derivativeGain = 0.25,
  });
  rollHold.SetEnabled(true);

  const auto command = rollHold.Update(aircraft, simulation.GetTickSizeSec());
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
  auto &pitchHold = simulation.GetAutopilot().GetPitchHoldController();

  pitchHold.SetEnabled(false);
  Require(!pitchHold.Update(aircraft, simulation.GetTickSizeSec()).has_value(),
      "Disabled pitch hold should not produce elevator command");

  const auto &properties = aircraft.GetProperties();
  const double targetPitchRad = properties.Pitch().Rad() + 0.2;
  pitchHold.SetTrimElevator(0.1);
  pitchHold.SetSettings({
      .targetPitchRad = targetPitchRad,
      .proportionalGain = 0.5,
      .derivativeGain = 0.25,
  });
  pitchHold.SetEnabled(true);

  const auto command = pitchHold.Update(aircraft, simulation.GetTickSizeSec());
  Require(command.has_value(), "Enabled pitch hold produced no command");

  const double expectedElevator =
      0.1 - 0.5 * (targetPitchRad - properties.Pitch().Rad())
      + 0.25 * properties.Q().RadPerSec();
  RequireNear(*command,
      expectedElevator,
      SimTimeTolerance,
      "Pitch hold elevator command mismatch");
}

void TestAutopilotModeAppliesAutopilotControllerOutput() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  auto &aircraft = simulation.GetAircraft();
  auto &manualController = simulation.GetManualFlightControlController();
  auto &autopilot = simulation.GetAutopilot();
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
  simulation.SetFlightControlMode(control::FlightControlMode::Autopilot);

  const double expectedElevator =
      control::ClampControlAxisValue(control::ControlAxis::Elevator,
          trimResult->elevator
              - 0.5 * (pitchTargetRad - properties.Pitch().Rad()));
  const double expectedAileron =
      control::ClampControlAxisValue(control::ControlAxis::Aileron,
          trimResult->aileron
              + 0.5 * (rollTargetRad - properties.Roll().Rad()));

  Require(simulation.Update(), "Autopilot mode update failed");

  const control::ControlInput &actualInput = aircraft.GetAircraftControlInput();
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
  auto &manualController = simulation.GetManualFlightControlController();
  auto &autopilot = simulation.GetAutopilot();
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
  simulation.SetFlightControlMode(control::FlightControlMode::Autopilot);

  const double expectedElevator =
      control::ClampControlAxisValue(control::ControlAxis::Elevator,
          trimResult->elevator
              - 0.5 * (pitchTargetRad - properties.Pitch().Rad()));

  Require(simulation.Update(), "Pitch hold pass-through update failed");

  const control::ControlInput &actualInput = aircraft.GetAircraftControlInput();
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
  auto &manualController = simulation.GetManualFlightControlController();
  auto &autopilot = simulation.GetAutopilot();
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
  simulation.SetFlightControlMode(control::FlightControlMode::Autopilot);

  const double expectedAileron =
      control::ClampControlAxisValue(control::ControlAxis::Aileron,
          trimResult->aileron
              + 0.5 * (rollTargetRad - properties.Roll().Rad()));

  Require(simulation.Update(), "Roll hold pass-through update failed");

  const control::ControlInput &actualInput = aircraft.GetAircraftControlInput();
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
    TestPauseStopsTime();
    TestStepOnceAdvancesOneTick();
    TestResumeAdvancesTime();
    TestRestartUsesStoredInitialCondition();
    TestRestartWithInitialCondition();
    TestCaptureCurrentStateCanRestart();
    TestEngineStateInspection();
    TestInvalidInitialConditionFails();
    TestStateProvider();
    TestStartAppliesInitialTrim();
    TestInitialTrimIsStoredInAutopilot();
    TestAircraftAxisSettersClampFinalInput();
    TestAircraftSetAircraftControlInputClampsFinalInput();
    TestManualFlightControlControllerAppliesCommands();
    TestManualModeIgnoresAutopilotController();
    TestRollHoldControllerComputesAileronCommand();
    TestPitchHoldControllerComputesElevatorCommand();
    TestAutopilotModeAppliesAutopilotControllerOutput();
    TestPitchHoldOnlyPassesThroughManualLateralAxes();
    TestRollHoldOnlyPassesThroughManualLongitudinalAxes();
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

  return 0;
}
