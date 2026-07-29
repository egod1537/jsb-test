#pragma once

#include <string>

namespace JSBSim {
class FGFDMExec;

class FlightProperties {
public:
  explicit FlightProperties(FGFDMExec &fdmExec);

  double Get(const std::string &name) const;
  void Set(const std::string &name, double value);

  double GetSimTimeSec() const; // t: simulation time [s]
  void SetSimTimeSec(double value);

  double GetAltitudeAglFt() const; // h_AGL: altitude above ground level [ft]
  void SetAltitudeAglFt(double value);

  double GetCalibratedAirspeedKts() const; // V_CAS: calibrated airspeed [kt]
  void SetCalibratedAirspeedKts(double value);

  double GetTrueAirspeedKts() const; // V_TAS: true airspeed [kt]
  double GetTrueAirspeedMps() const; // V_TAS: true airspeed [m/s]

  double GetUMps() const; // u: body-axis forward velocity [m/s]
  double GetVMps() const; // v: body-axis lateral velocity [m/s]
  double GetWMps() const; // w: body-axis vertical velocity [m/s]

  double GetVerticalSpeedFps() const;      // h_dot: vertical speed [ft/s]
  double GetVerticalSpeedFtPerMin() const; // h_dot: vertical speed [ft/min]
  double GetVerticalSpeedMps() const;      // h_dot: vertical speed [m/s]

  double GetPitchRad() const; // theta: pitch angle [rad]
  void SetPitchRad(double value);

  double GetRollDeg() const;  // phi: roll angle [deg]
  double GetPitchDeg() const; // theta: pitch angle [deg]

  double GetAlphaDeg() const; // alpha: angle of attack [deg]
  double GetBetaDeg() const;  // beta: sideslip angle [deg]

  double GetPDegPerSec() const; // p: roll rate [deg/s]
  double GetQDegPerSec() const; // q: pitch rate [deg/s]
  double GetRDegPerSec() const; // r: yaw rate [deg/s]

  double GetUDotMps2() const; // u_dot: body-axis forward acceleration [m/s^2]
  double GetVDotMps2() const; // v_dot: body-axis lateral acceleration [m/s^2]
  double GetWDotMps2() const; // w_dot: body-axis vertical acceleration [m/s^2]

  double GetPdotDegPerSec2() const; // p_dot: roll acceleration [deg/s^2]
  double GetQdotDegPerSec2() const; // q_dot: pitch acceleration [deg/s^2]
  double GetRdotDegPerSec2() const; // r_dot: yaw acceleration [deg/s^2]

private:
  FGFDMExec &fdmExec_;
};
} // namespace JSBSim
