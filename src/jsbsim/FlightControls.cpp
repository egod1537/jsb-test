#include "jsbsim/FlightControls.hpp"

#include <FGFDMExec.h>

namespace {
constexpr const char *ElevatorCommand = "fcs/elevator-cmd-norm";
constexpr const char *PitchTrimCommand = "fcs/pitch-trim-cmd-norm";
constexpr const char *AileronCommand = "fcs/aileron-cmd-norm";
constexpr const char *RudderCommand = "fcs/rudder-cmd-norm";
constexpr const char *ThrottleCommand = "fcs/throttle-cmd-norm";
} // namespace

namespace JSBSim {
FlightControls::FlightControls(FGFDMExec &fdmExec) : fdmExec_(fdmExec) {}

double FlightControls::GetElevator() const {
  return fdmExec_.GetPropertyValue(ElevatorCommand);
}

void FlightControls::SetElevator(double value) {
  fdmExec_.SetPropertyValue(ElevatorCommand, value);
}

double FlightControls::GetPitchTrim() const {
  return fdmExec_.GetPropertyValue(PitchTrimCommand);
}

void FlightControls::SetPitchTrim(double value) {
  fdmExec_.SetPropertyValue(PitchTrimCommand, value);
}

double FlightControls::GetAileron() const {
  return fdmExec_.GetPropertyValue(AileronCommand);
}

void FlightControls::SetAileron(double value) {
  fdmExec_.SetPropertyValue(AileronCommand, value);
}

double FlightControls::GetRudder() const {
  return fdmExec_.GetPropertyValue(RudderCommand);
}

void FlightControls::SetRudder(double value) {
  fdmExec_.SetPropertyValue(RudderCommand, value);
}

double FlightControls::GetThrottle() const {
  return fdmExec_.GetPropertyValue(ThrottleCommand);
}

void FlightControls::SetThrottle(double value) {
  fdmExec_.SetPropertyValue(ThrottleCommand, value);
}
} // namespace JSBSim
