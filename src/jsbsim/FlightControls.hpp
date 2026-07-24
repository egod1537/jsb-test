#pragma once

namespace JSBSim {
class FGFDMExec;

class FlightControls {
public:
  explicit FlightControls(FGFDMExec &fdmExec);

  double GetElevator() const;
  void SetElevator(double value);

  double GetPitchTrim() const;
  void SetPitchTrim(double value);

  double GetAileron() const;
  void SetAileron(double value);

  double GetRudder() const;
  void SetRudder(double value);

  double GetThrottle() const;
  void SetThrottle(double value);

private:
  FGFDMExec &fdmExec_;
};
} // namespace JSBSim
