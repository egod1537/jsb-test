#pragma once

#include "application/sim/control/ControlInput.hpp"
#include "application/sim/control/ControlInputStrategy.hpp"
#include "application/sim/jsbsim/FlightControls.hpp"
#include "application/sim/jsbsim/FlightProperties.hpp"
#include "application/sim/AircraftState.hpp"
#include "application/sim/EngineState.hpp"
#include "application/sim/InitialCondition.hpp"
#include "application/sim/SimulationConfig.h"

#include <cstddef>
#include <memory>
#include <vector>

namespace JSBSim {
class FGFDMExec;
} // namespace JSBSim

namespace sim {
class Aircraft {
public:
  Aircraft();
  ~Aircraft();

  Aircraft(const Aircraft &) = delete;
  Aircraft &operator=(const Aircraft &) = delete;

  bool Initialize(const SimulationConfig &config);
  bool Step();
  bool Update();

  AircraftState GetAircraftState() const;
  AircraftStateDerivative GetAircraftStateDerivative() const;

  bool ApplyInitialCondition(const InitialCondition &initialCondition);
  void SetInitialConditionInputs(const InitialCondition &initialCondition);
  InitialCondition CaptureCurrentCondition() const;
  bool Reset(const SimulationConfig &config,
      const InitialCondition &initialCondition);
  void ResetControlInput();

  JSBSim::FGFDMExec &GetFDMExec();
  const JSBSim::FGFDMExec &GetFDMExec() const;

  JSBSim::FlightProperties &GetProperties();
  const JSBSim::FlightProperties &GetProperties() const;

  JSBSim::FlightControls &GetFlightControls();
  const JSBSim::FlightControls &GetFlightControls() const;

  const control::ControlInput &GetAircraftControlInput() const;
  void SetAircraftControlInput(const control::ControlInput &input);

  const control::ControlInput &GetControlInput() const;
  control::ControlInputStrategy &GetControlInputStrategy();
  const control::ControlInputStrategy &GetControlInputStrategy() const;
  void SetControlInputStrategy(
      std::unique_ptr<control::ControlInputStrategy> strategy);
  void UseManualControlInputStrategy();

  bool SetControlInputCommand(const control::ControlInput &input);
  bool SetControlInputCommand(control::ControlAxis axis, double value);
  bool AdjustControlInputCommand(control::ControlAxis axis, double delta);
  bool SetElevatorInput(double value);
  bool SetAileronInput(double value);
  bool SetRudderInput(double value);
  bool SetThrottleInput(double value);

  std::size_t GetEngineCount() const;
  EngineState GetEngineState(std::size_t index) const;
  std::vector<EngineState> GetEngineStates() const;
  bool IsAnyEngineRunning() const;
  bool AreAllEnginesRunning() const;

private:
  void ConfigurePaths();
  bool LoadAircraft(const SimulationConfig &config);
  void ConfigureSimulation(const SimulationConfig &config);
  void ConfigureInitialConditions(const SimulationConfig &config);
  bool InitializeState();
  bool UpdateControlInput();
  void ApplyControlInput();

  std::unique_ptr<JSBSim::FGFDMExec> fdm_;
  JSBSim::FlightProperties properties_;
  JSBSim::FlightControls flightControls_;
  std::unique_ptr<control::ControlInputStrategy> controlInputStrategy_;
  control::ControlInput controlInput_;
  double controlDt_ = 0.0;
};
} // namespace sim
