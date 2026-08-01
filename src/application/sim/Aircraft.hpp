#pragma once

#include "application/sim/jsbsim/ControlSystem.hpp"
#include "application/sim/jsbsim/EngineSystem.hpp"
#include "application/sim/jsbsim/Properties.hpp"
#include "application/sim/AircraftState.hpp"
#include "application/sim/InitialCondition.hpp"
#include "application/sim/SimulationConfig.h"

#include <memory>

namespace JSBSim {
class FGFDMExec;
} // namespace JSBSim

namespace gnc {
enum class TrimMode;
struct TrimRequest;
} // namespace gnc

namespace sim {
class Aircraft {
public:
  // Lifetime and stepping
  Aircraft();
  ~Aircraft();
  Aircraft(const Aircraft &) = delete;
  Aircraft &operator=(const Aircraft &) = delete;
  bool Initialize(const SimulationConfig &config,
      const InitialCondition &initialCondition);
  bool Tick();

  // Initial condition and reset
  bool ApplyInitialCondition(const InitialCondition &initialCondition);
  void SetInitialConditionInputs(const InitialCondition &initialCondition);
  InitialCondition GetCurrentCondition() const;
  bool Reset(const SimulationConfig &config,
      const InitialCondition &initialCondition);
  void ResetSimulationTime();

  // Trim operations
  bool ApplyTrimInitialCondition(const gnc::TrimRequest &request);
  void ExecuteTrim(gnc::TrimMode mode);

  // Aircraft state
  AircraftState GetAircraftState() const;
  AircraftStateDerivative GetAircraftStateDerivative() const;

  // Flight model interfaces
  jsbsim::ControlSystem &GetControls();
  const jsbsim::ControlSystem &GetControls() const;
  jsbsim::EngineSystem &GetEngines();
  const jsbsim::EngineSystem &GetEngines() const;
  jsbsim::Properties &GetProperties();
  const jsbsim::Properties &GetProperties() const;

private:
  // JSBSim setup
  void ConfigurePaths();
  bool LoadAircraft(const SimulationConfig &config);
  void ConfigureSimulation(const SimulationConfig &config);
  bool InitializeState();

  // JSBSim dependencies
  std::unique_ptr<JSBSim::FGFDMExec> fdm_;
  jsbsim::ControlSystem controls_;
  jsbsim::EngineSystem engines_;
  jsbsim::Properties properties_;
};
} // namespace sim
