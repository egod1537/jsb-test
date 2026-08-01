#include "application/flightgear/FlightGearSystem.hpp"

#include "application/sim/Aircraft.hpp"

#include <iostream>
#include <memory>
#include <utility>

namespace flightgear {
bool FlightGearSystem::Initialize() {
  if (sender_ != nullptr) {
    return true;
  }

  auto sender = std::make_unique<FlightGearSender>();
  if (!sender->IsOpen()) {
    std::cerr << "Failed to initialize FlightGear sender.\n";
    return false;
  }

  sender_ = std::move(sender);
  return true;
}

void FlightGearSystem::Update(const sim::Aircraft &aircraft) {
  if (sender_ != nullptr && !sender_->Send(aircraft.GetProperties())) {
    std::cerr << "Failed to send FlightGear packet\n";
  }
}

void FlightGearSystem::Shutdown() { sender_.reset(); }
} // namespace flightgear
