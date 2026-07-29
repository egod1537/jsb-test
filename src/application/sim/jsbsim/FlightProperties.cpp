#include "application/sim/jsbsim/FlightProperties.hpp"

#include <FGFDMExec.h>

namespace {
constexpr const char *SimTimeSec = "simulation/sim-time-sec";
constexpr const char *AltitudeAglFt = "position/h-agl-ft";
constexpr const char *CalibratedAirspeedKts = "velocities/vc-kts";
constexpr const char *TrueAirspeedKts = "velocities/vtrue-kts";
constexpr const char *TrueAirspeedFps = "velocities/vtrue-fps";
constexpr const char *UFps = "velocities/u-fps";
constexpr const char *VFps = "velocities/v-fps";
constexpr const char *WFps = "velocities/w-fps";
constexpr const char *VerticalSpeedFps = "velocities/h-dot-fps";
constexpr const char *RollRad = "attitude/roll-rad";
constexpr const char *PitchRad = "attitude/pitch-rad";
constexpr const char *AlphaDeg = "aero/alpha-deg";
constexpr const char *BetaDeg = "aero/beta-deg";
constexpr const char *RollRateRadPerSec = "velocities/p-rad_sec";
constexpr const char *PitchRateRadPerSec = "velocities/q-rad_sec";
constexpr const char *YawRateRadPerSec = "velocities/r-rad_sec";
constexpr const char *UDotFtPerSec2 = "accelerations/udot-ft_sec2";
constexpr const char *VDotFtPerSec2 = "accelerations/vdot-ft_sec2";
constexpr const char *WDotFtPerSec2 = "accelerations/wdot-ft_sec2";
constexpr const char *PdotRadPerSec2 = "accelerations/pdot-rad_sec2";
constexpr const char *QdotRadPerSec2 = "accelerations/qdot-rad_sec2";
constexpr const char *RdotRadPerSec2 = "accelerations/rdot-rad_sec2";

constexpr double FeetToMeters = 0.3048;
constexpr double RadToDeg = 57.295779513082320876;

double FeetPerSecToMetersPerSec(double value) { return value * FeetToMeters; }
double FeetPerSec2ToMetersPerSec2(double value) { return value * FeetToMeters; }
double RadToDegValue(double value) { return value * RadToDeg; }
} // namespace

namespace JSBSim {
FlightProperties::FlightProperties(FGFDMExec &fdmExec)
    : fdmExec_(fdmExec) {}

double FlightProperties::Get(const std::string &name) const {
  return fdmExec_.GetPropertyValue(name);
}

void FlightProperties::Set(const std::string &name, double value) {
  fdmExec_.SetPropertyValue(name, value);
}

double FlightProperties::GetSimTimeSec() const { return Get(SimTimeSec); }

void FlightProperties::SetSimTimeSec(double value) { Set(SimTimeSec, value); }

double FlightProperties::GetAltitudeAglFt() const {
  return Get(AltitudeAglFt);
}

void FlightProperties::SetAltitudeAglFt(double value) {
  Set(AltitudeAglFt, value);
}

double FlightProperties::GetCalibratedAirspeedKts() const {
  return Get(CalibratedAirspeedKts);
}

void FlightProperties::SetCalibratedAirspeedKts(double value) {
  Set(CalibratedAirspeedKts, value);
}

double FlightProperties::GetTrueAirspeedKts() const {
  return Get(TrueAirspeedKts);
}

double FlightProperties::GetTrueAirspeedMps() const {
  return FeetPerSecToMetersPerSec(Get(TrueAirspeedFps));
}

double FlightProperties::GetUMps() const {
  return FeetPerSecToMetersPerSec(Get(UFps));
}

double FlightProperties::GetVMps() const {
  return FeetPerSecToMetersPerSec(Get(VFps));
}

double FlightProperties::GetWMps() const {
  return FeetPerSecToMetersPerSec(Get(WFps));
}

double FlightProperties::GetVerticalSpeedFps() const {
  return Get(VerticalSpeedFps);
}

double FlightProperties::GetVerticalSpeedFtPerMin() const {
  return GetVerticalSpeedFps() * 60.0;
}

double FlightProperties::GetVerticalSpeedMps() const {
  return FeetPerSecToMetersPerSec(GetVerticalSpeedFps());
}

double FlightProperties::GetPitchRad() const { return Get(PitchRad); }

void FlightProperties::SetPitchRad(double value) { Set(PitchRad, value); }

double FlightProperties::GetRollDeg() const {
  return RadToDegValue(Get(RollRad));
}

double FlightProperties::GetPitchDeg() const {
  return RadToDegValue(GetPitchRad());
}

double FlightProperties::GetAlphaDeg() const { return Get(AlphaDeg); }

double FlightProperties::GetBetaDeg() const { return Get(BetaDeg); }

double FlightProperties::GetPDegPerSec() const {
  return RadToDegValue(Get(RollRateRadPerSec));
}

double FlightProperties::GetQDegPerSec() const {
  return RadToDegValue(Get(PitchRateRadPerSec));
}

double FlightProperties::GetRDegPerSec() const {
  return RadToDegValue(Get(YawRateRadPerSec));
}

double FlightProperties::GetUDotMps2() const {
  return FeetPerSec2ToMetersPerSec2(Get(UDotFtPerSec2));
}

double FlightProperties::GetVDotMps2() const {
  return FeetPerSec2ToMetersPerSec2(Get(VDotFtPerSec2));
}

double FlightProperties::GetWDotMps2() const {
  return FeetPerSec2ToMetersPerSec2(Get(WDotFtPerSec2));
}

double FlightProperties::GetPdotDegPerSec2() const {
  return RadToDegValue(Get(PdotRadPerSec2));
}

double FlightProperties::GetQdotDegPerSec2() const {
  return RadToDegValue(Get(QdotRadPerSec2));
}

double FlightProperties::GetRdotDegPerSec2() const {
  return RadToDegValue(Get(RdotRadPerSec2));
}
} // namespace JSBSim
