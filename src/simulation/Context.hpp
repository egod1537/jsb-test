#pragma once

#include "simulation/SimulationConfig.h"

#include <optional>
#include <string>

namespace sim {
class Aircraft;

class Context {
public:
  Context(Aircraft &aircraft, const SimulationConfig &config,
      std::optional<std::string> *lastError = nullptr);

  Aircraft &GetAircraft() const;
  const SimulationConfig &GetConfig() const;
  double GetTickSizeSec() const;
  void SetError(std::string message) const;

private:
  Aircraft *aircraft_ = nullptr;
  const SimulationConfig *config_ = nullptr;
  std::optional<std::string> *lastError_ = nullptr;
};
} // namespace sim
