#include "simulation/Aircraft.hpp"
#include "simulation/Simulation.hpp"
#include "state/IStateProvider.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
constexpr double SimTimeTolerance = 1.0e-9;
constexpr double AltitudeToleranceFt = 1.0;
constexpr double AirspeedToleranceKts = 0.5;
constexpr double HeadingToleranceDeg = 0.5;

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
}

void TestManualControlInputStrategyAppliesCommands() {
  sim::Simulation simulation;
  StartSimulation(simulation);
  auto &aircraft = simulation.GetAircraft();

  aircraft.SetAircraftControlInput({
      .elevator = 0.25,
      .aileron = 2.0,
      .rudder = -0.25,
      .throttle = 0.5,
  });

  const control::ControlInput &commandedInput =
      aircraft.GetControlInputStrategy().GetCommandedInput();
  RequireNear(commandedInput.throttle,
      0.5,
      SimTimeTolerance,
      "Throttle command mismatch");
  RequireNear(commandedInput.aileron,
      1.0,
      SimTimeTolerance,
      "Aileron command clamp");
  RequireNear(commandedInput.elevator,
      0.25,
      SimTimeTolerance,
      "Elevator command mismatch");
  RequireNear(commandedInput.rudder,
      -0.25,
      SimTimeTolerance,
      "Rudder command mismatch");

  const control::ControlInput &immediateInput =
      aircraft.GetAircraftControlInput();
  RequireNear(immediateInput.throttle,
      0.5,
      SimTimeTolerance,
      "Throttle aggregate input was not applied immediately");
  RequireNear(immediateInput.aileron,
      1.0,
      SimTimeTolerance,
      "Aileron aggregate input was not applied immediately");

  Require(simulation.Update(), "Control strategy update failed");

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
    TestManualControlInputStrategyAppliesCommands();
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

  return 0;
}
