#pragma once

#include "application/sim/FDMState.hpp"
#include "application/sim/InitialCondition.hpp"
#include "application/sim/SimulationConfig.h"
#include "application/sim/linearizer/LinearizationResult.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace sim {
class AsyncAircraftLinearizer {
public:
  struct Completion {
    std::uint64_t generation{};
    std::optional<gnc::LinearizationResult> linearization;
    std::string errorMessage;
  };

  AsyncAircraftLinearizer();
  ~AsyncAircraftLinearizer();

  AsyncAircraftLinearizer(const AsyncAircraftLinearizer &other) = delete;
  AsyncAircraftLinearizer &operator=(
      const AsyncAircraftLinearizer &other) = delete;

  bool Submit(std::uint64_t generation, const SimulationConfig &config,
      const InitialCondition &initialCondition, FDMState sourceState);
  bool IsBusy() const;
  std::optional<Completion> TakeCompletion();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace sim
