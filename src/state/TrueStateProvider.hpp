#pragma once

#include "state/IStateProvider.hpp"

namespace sim {
class Aircraft;
}

namespace state {
class TrueStateProvider final : public IStateProvider {
public:
  explicit TrueStateProvider(const sim::Aircraft &aircraft);

  AircraftState GetState() const override;

private:
  const sim::Aircraft &aircraft_;
};
} // namespace state
