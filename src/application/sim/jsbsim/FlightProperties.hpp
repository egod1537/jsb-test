#pragma once

#include <string>

namespace JSBSim {
class FGFDMExec;
class FlightProperties;

class TimeView {
public:
  TimeView(const FlightProperties &properties, const char *secPath);

  double Sec() const;

private:
  const FlightProperties &properties_;
  const char *secPath_;
};

class MutableTimeView {
public:
  MutableTimeView(FlightProperties &properties, const char *secPath);

  double Sec() const;
  void SetSec(double value) const;

private:
  FlightProperties &properties_;
  const char *secPath_;
};

class DistanceView {
public:
  DistanceView(const FlightProperties &properties, const char *ftPath);

  double Ft() const;

private:
  const FlightProperties &properties_;
  const char *ftPath_;
};

class MutableDistanceView {
public:
  MutableDistanceView(FlightProperties &properties, const char *ftPath);

  double Ft() const;
  void SetFt(double value) const;

private:
  FlightProperties &properties_;
  const char *ftPath_;
};

class AngleView {
public:
  AngleView(const FlightProperties &properties, const char *radPath,
      const char *degPath);

  double Rad() const;
  double Deg() const;

private:
  const FlightProperties &properties_;
  const char *radPath_;
  const char *degPath_;
};

class MutableAngleView {
public:
  MutableAngleView(FlightProperties &properties, const char *radPath,
      const char *degPath);

  double Rad() const;
  double Deg() const;
  void SetRad(double value) const;
  void SetDeg(double value) const;

private:
  FlightProperties &properties_;
  const char *radPath_;
  const char *degPath_;
};

class AngularRateView {
public:
  AngularRateView(const FlightProperties &properties,
      const char *rateRadPerSecPath, const char *dotRadPerSec2Path);

  double RadPerSec() const;
  double DegPerSec() const;
  double DotRadPerSec2() const;
  double DotDegPerSec2() const;

private:
  const FlightProperties &properties_;
  const char *rateRadPerSecPath_;
  const char *dotRadPerSec2Path_;
};

class LinearVelocityView {
public:
  LinearVelocityView(const FlightProperties &properties,
      const char *velocityFpsPath, const char *dotFps2Path);

  double Mps() const;
  double DotMps2() const;

private:
  const FlightProperties &properties_;
  const char *velocityFpsPath_;
  const char *dotFps2Path_;
};

class SpeedView {
public:
  SpeedView(const FlightProperties &properties, const char *fpsPath,
      const char *ktsPath);

  double Mps() const;
  double Kts() const;
  double Fps() const;
  double FtPerMin() const;

private:
  const FlightProperties &properties_;
  const char *fpsPath_;
  const char *ktsPath_;
};

class MutableSpeedView {
public:
  MutableSpeedView(FlightProperties &properties, const char *fpsPath,
      const char *ktsPath);

  double Mps() const;
  double Kts() const;
  double Fps() const;
  double FtPerMin() const;
  void SetKts(double value) const;

private:
  FlightProperties &properties_;
  const char *fpsPath_;
  const char *ktsPath_;
};

class FlightProperties {
public:
  explicit FlightProperties(FGFDMExec &fdmExec);

  double Get(const std::string &name) const;
  void Set(const std::string &name, double value);

  MutableTimeView SimTime();
  TimeView SimTime() const;

  MutableDistanceView AltitudeAgl();
  DistanceView AltitudeAgl() const;

  MutableSpeedView CalibratedAirspeed();
  SpeedView CalibratedAirspeed() const;
  SpeedView TrueAirspeed() const;
  SpeedView VerticalSpeed() const;

  LinearVelocityView U() const;
  LinearVelocityView V() const;
  LinearVelocityView W() const;

  MutableAngleView Roll();
  AngleView Roll() const;
  MutableAngleView Pitch();
  AngleView Pitch() const;
  AngleView Alpha() const;
  AngleView Beta() const;

  AngularRateView P() const;
  AngularRateView Q() const;
  AngularRateView R() const;

private:
  FGFDMExec &fdmExec_;
};
} // namespace JSBSim
