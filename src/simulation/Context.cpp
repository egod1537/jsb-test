#include "simulation/Context.hpp"

#include "simulation/Aircraft.hpp"

#include <utility>

namespace sim {
Context::Context(Aircraft &aircraft, const SimulationConfig &config,
    std::optional<std::string> *lastError)
    : aircraft_(&aircraft), config_(&config), lastError_(lastError) {}

Aircraft &Context::GetAircraft() const { return *aircraft_; }

const SimulationConfig &Context::GetConfig() const { return *config_; }

double Context::GetTickSizeSec() const { return config_->GetDT(); }

void Context::SetError(std::string message) const {
  if (lastError_ != nullptr) {
    *lastError_ = std::move(message);
  }
}
} // namespace sim
