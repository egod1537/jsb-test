#pragma once

#include "state/AircraftState.hpp"

namespace state {
class IStateProvider {
public:
  virtual ~IStateProvider() = default;

  virtual AircraftState GetState() const = 0;
};
} // namespace state
