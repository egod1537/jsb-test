#pragma once

#include "control/KeyboardInputSystem.hpp"
#include "flightgear/FlightGearSystem.hpp"
#include "gnc/RollHoldSystem.hpp"
#include "simulation/InitialCondition.hpp"
#include "simulation/Aircraft.hpp"
#include "simulation/SimulationConfig.h"
#include "simulation/System.hpp"
#include "simulation/Tick.hpp"
#include "state/IStateProvider.hpp"
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sim {
class Context;

enum class SimulationState {
  Running,
  Paused,
  Stopped,
};

const char *ToString(SimulationState state);

class Simulation {
public:
  Simulation();
  ~Simulation();

  Simulation(const Simulation &) = delete;
  Simulation &operator=(const Simulation &) = delete;

  bool Start(const SimulationConfig &);
  bool Update();
  void Exit();

  bool Initialize(const SimulationConfig &);

  const SimulationConfig &GetConfig() const { return config_; }
  double GetTickSizeSec() const { return config_.GetDT(); }

  SimulationState GetState() const { return state_; }
  bool IsRunning() const { return state_ == SimulationState::Running; }
  bool IsPaused() const { return state_ == SimulationState::Paused; }
  bool IsStopped() const { return state_ == SimulationState::Stopped; }

  void Pause();
  void Resume();
  void TogglePause();

  bool RequestStep();
  bool StepOnce();
  std::uint32_t GetPendingStepCount() const { return pendingSteps_; }

  bool SetInitialCondition(const InitialCondition &initialCondition);
  bool Restart();
  bool Restart(const InitialCondition &initialCondition);
  InitialCondition CaptureCurrentCondition() const;
  const InitialCondition &GetInitialCondition() const {
    return initialCondition_;
  }
  const InitialCondition &GetDefaultInitialCondition() const {
    return defaultInitialCondition_;
  }
  const std::optional<std::string> &GetLastError() const { return lastError_; }
  void ClearLastError() { lastError_.reset(); }

  Aircraft &GetAircraft();
  const Aircraft &GetAircraft() const;
  const state::IStateProvider &GetStateProvider() const;

private:
  bool AdvanceOneTick();
  Context MakeContext();
  bool InitializeSystems();
  bool ResetSystems();
  bool RunPreStepSystems(const Tick &tick);
  bool RunPostStepSystems(const Tick &tick);
  void ShutdownSystems();
  void RegisterSystems();
  Tick MakeTick() const;
  void ResetLogTimer();
  void PrintState() const;
  bool ValidateInitialCondition(const InitialCondition &initialCondition);
  void SetError(std::string message);

  Aircraft aircraft_;
  std::unique_ptr<state::IStateProvider> stateProvider_;
  control::KeyboardInputSystem keyboard_;
  gnc::RollHoldSystem rollHold_;
  flightgear::FlightGearSystem flightGear_;
  std::vector<sim::System *> systems_;

  SimulationConfig config_;
  InitialCondition defaultInitialCondition_;
  InitialCondition initialCondition_;
  std::optional<std::string> lastError_;
  std::uint32_t pendingSteps_ = 0;
  std::uint64_t tickIndex_ = 0;
  double nextLogTime_ = 0.0;
  bool started_ = false;
  SimulationState state_ = SimulationState::Stopped;
};
} // namespace sim
