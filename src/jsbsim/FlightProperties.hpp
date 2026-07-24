#pragma once

#include <string>

namespace JSBSim {
class FGFDMExec;

class FlightProperties {
public:
  explicit FlightProperties(FGFDMExec &fdmExec);

  double Get(const std::string &name) const;
  void Set(const std::string &name, double value);

  double GetSimTimeSec() const;
  void SetSimTimeSec(double value);

  double GetAltitudeAglFt() const;
  void SetAltitudeAglFt(double value);

  double GetCalibratedAirspeedKts() const;
  void SetCalibratedAirspeedKts(double value);

  double GetTrueAirspeedKts() const;
  double GetTrueAirspeedMps() const;

  double GetUMps() const;
  double GetVMps() const;
  double GetWMps() const;

  double GetVerticalSpeedFps() const;
  double GetVerticalSpeedFtPerMin() const;
  double GetVerticalSpeedMps() const;

  double GetPitchRad() const;
  void SetPitchRad(double value);

  double GetRollDeg() const;
  double GetPitchDeg() const;

  double GetAlphaDeg() const;
  double GetBetaDeg() const;

  double GetPDegPerSec() const;
  double GetQDegPerSec() const;
  double GetRDegPerSec() const;

  double GetUDotMps2() const;
  double GetVDotMps2() const;
  double GetWDotMps2() const;

  double GetPdotDegPerSec2() const;
  double GetQdotDegPerSec2() const;
  double GetRdotDegPerSec2() const;

private:
  FGFDMExec &fdmExec_;
};
} // namespace JSBSim
