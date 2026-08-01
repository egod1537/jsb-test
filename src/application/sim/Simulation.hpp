#pragma once

#include "application/sim/Aircraft.hpp"
#include "application/sim/Component.hpp"
#include "application/sim/ErrorTracker.hpp"
#include "application/sim/InitialCondition.hpp"
#include "application/sim/SimulationConfig.h"
#include "application/sim/Tick.hpp"
#include "application/sim/control/FlightControlManager.hpp"

#include <cstdint>
#include <memory>
#include <type_traits>
#include <typeinfo>
#include <utility>
#include <vector>

namespace sim {
class Simulation {
public:
  // Lifetime and stepping
  Simulation();
  ~Simulation();

  Simulation(const Simulation &) = delete;
  Simulation &operator=(const Simulation &) = delete;

  bool Initialize(const SimulationConfig &);
  bool Tick();
  void Shutdown();

  // Configuration
  const SimulationConfig &GetConfig() const;
  double GetTickSizeSec() const;

  // Initial condition
  bool Reset();
  bool Reset(const InitialCondition &initialCondition);
  InitialCondition GetCurrentCondition() const;
  const InitialCondition &GetDefaultInitialCondition() const;

  // Aircraft
  Aircraft &GetAircraft();
  const Aircraft &GetAircraft() const;

  // Components
  template <typename T, typename... Args> T *AddComponent(Args &&...args);
  template <typename T> T *GetComponent();
  template <typename T> const T *GetComponent() const;
  template <typename T> bool RemoveComponent();

  // Diagnostics
  ErrorTracker &GetErrorTracker();
  const ErrorTracker &GetErrorTracker() const;

private:
  // Tick processing
  bool ProcessTick();
  sim::Tick MakeTick() const;

  // Initial condition
  bool ApplyInitialTrim(const InitialCondition &initialCondition);

  // Components
  bool InitializeComponent(Component &component);
  bool InitializeComponents();
  bool ResetComponents();
  bool RunPreTickComponents(const sim::Tick &tick);
  bool TickComponents(const sim::Tick &tick);
  bool RunPostTickComponents(const sim::Tick &tick);
  void ShutdownComponents();
  Component *FindComponent(const std::type_info &type);
  const Component *FindComponent(const std::type_info &type) const;

  // Configuration
  SimulationConfig config_;

  // Runtime state
  bool initialized_ = false;

  // Initial condition
  InitialCondition defaultInitialCondition_;

  // Simulation clock
  std::uint64_t tickIndex_ = 0;

  // Aircraft state
  Aircraft aircraft_;

  // Components
  std::vector<std::unique_ptr<Component>> components_;

  // Diagnostics
  ErrorTracker errorTracker_;

  friend class Component;
};
} // namespace sim

#include "application/sim/Simulation.inl"
