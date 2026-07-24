#pragma once

#include "control/ControlInput.hpp"
#include "gnc/TrimTypes.hpp"
#include "jsbsim/FlightControls.hpp"
#include "jsbsim/FlightProperties.hpp"
#include "simulation/SimulationConfig.h"

#include <memory>

namespace JSBSim {
class FGFDMExec;
} // namespace JSBSim

namespace sim {
class FlightDynamics {
public:
  FlightDynamics();
  ~FlightDynamics();

  FlightDynamics(const FlightDynamics &) = delete;
  FlightDynamics &operator=(const FlightDynamics &) = delete;

  bool Initialize(const SimulationConfig &config);
  bool Update();

  JSBSim::FGFDMExec &GetFDMExec();
  const JSBSim::FGFDMExec &GetFDMExec() const;

  JSBSim::FlightProperties &GetProperties();
  const JSBSim::FlightProperties &GetProperties() const;

  JSBSim::FlightControls &GetFlightControls();
  const JSBSim::FlightControls &GetFlightControls() const;

  control::ControlInput &GetControlInput();
  const control::ControlInput &GetControlInput() const;

  void SetElevatorInput(double value);
  void SetAileronInput(double value);
  void SetRudderInput(double value);
  void SetThrottleInput(double value);

  gnc::TrimResult Trim(const gnc::TrimRequest &request);
  gnc::TrimResult TrimCurrentState(
      gnc::TrimMode mode = gnc::TrimMode::Longitudinal);

private:
  void ConfigurePaths();
  bool LoadAircraft(const SimulationConfig &config);
  void ConfigureSimulation(const SimulationConfig &config);
  void ConfigureInitialConditions(const SimulationConfig &config);
  bool InitializeState();
  void ApplyControlInput();
  void ApplyTrimRequestInitialConditions(const gnc::TrimRequest &request);
  void ApplyCurrentStateInitialConditions();
  void PreparePropulsionForTrim(gnc::TrimMode mode);
  gnc::TrimResult ExecuteTrim(gnc::TrimMode mode);
  gnc::TrimResult BuildTrimResult() const;
  void ApplyTrimResultToControlInput(const gnc::TrimResult &result);

  static int ToJSBTrimMode(gnc::TrimMode mode);

  std::unique_ptr<JSBSim::FGFDMExec> fdm_;
  JSBSim::FlightProperties properties_;
  JSBSim::FlightControls flightControls_;
  control::ControlInput controlInput_;
};
} // namespace sim
