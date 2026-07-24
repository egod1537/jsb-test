#pragma once

#include <memory>

namespace JSBSim {
class FGFDMExec;
}

namespace flightgear {
class FlightGearSender {
public:
  FlightGearSender();
  ~FlightGearSender();

  FlightGearSender(const FlightGearSender &) = delete;
  FlightGearSender &operator=(const FlightGearSender &) = delete;

  bool IsOpen() const;
  bool Send(JSBSim::FGFDMExec &fdm);

private:
  class Impl;
  std::unique_ptr<Impl> m_Impl;
};
} // namespace flightgear
