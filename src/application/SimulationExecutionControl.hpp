#pragma once

#include <cstdint>

namespace sim {
struct InitialCondition;
}

namespace application {
enum class SimulationExecutionState {
  Running,
  Paused,
  Stopped,
};

inline const char *ToString(SimulationExecutionState state) {
  switch (state) {
  case SimulationExecutionState::Running:
    return "Running";
  case SimulationExecutionState::Paused:
    return "Paused";
  case SimulationExecutionState::Stopped:
    return "Stopped";
  }

  return "Unknown";
}

class SimulationExecutionControl {
public:
  virtual ~SimulationExecutionControl() = default;

  virtual SimulationExecutionState GetSimulationExecutionState() const = 0;
  virtual void PauseSimulation() = 0;
  virtual void ResumeSimulation() = 0;
  virtual void RequestSimulationTick() = 0;
  virtual bool ResetSimulation() = 0;
  virtual bool ResetSimulation(
      const sim::InitialCondition &initialCondition) = 0;
  virtual std::uint32_t GetPendingSimulationTickCount() const = 0;
};
} // namespace application
