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
constexpr double KnotToFeetPerSec = 1.6878098571011957;
constexpr double RadToDeg = 57.295779513082320876;
constexpr double DegToRad = 0.017453292519943295769;

double FeetPerSecToMetersPerSec(double value) { return value * FeetToMeters; }
double FeetPerSec2ToMetersPerSec2(double value) { return value * FeetToMeters; }
double FeetPerSecToKts(double value) { return value / KnotToFeetPerSec; }
double KtsToFeetPerSec(double value) { return value * KnotToFeetPerSec; }
double RadToDegValue(double value) { return value * RadToDeg; }
double DegToRadValue(double value) { return value * DegToRad; }
} // namespace

namespace JSBSim {
TimeView::TimeView(const FlightProperties &properties, const char *secPath)
    : properties_(properties), secPath_(secPath) {}

double TimeView::Sec() const { return properties_.Get(secPath_); }

MutableTimeView::MutableTimeView(FlightProperties &properties,
    const char *secPath)
    : properties_(properties), secPath_(secPath) {}

double MutableTimeView::Sec() const { return properties_.Get(secPath_); }

void MutableTimeView::SetSec(double value) const {
  properties_.Set(secPath_, value);
}

DistanceView::DistanceView(const FlightProperties &properties,
    const char *ftPath)
    : properties_(properties), ftPath_(ftPath) {}

double DistanceView::Ft() const { return properties_.Get(ftPath_); }

MutableDistanceView::MutableDistanceView(FlightProperties &properties,
    const char *ftPath)
    : properties_(properties), ftPath_(ftPath) {}

double MutableDistanceView::Ft() const { return properties_.Get(ftPath_); }

void MutableDistanceView::SetFt(double value) const {
  properties_.Set(ftPath_, value);
}

AngleView::AngleView(const FlightProperties &properties, const char *radPath,
    const char *degPath)
    : properties_(properties), radPath_(radPath), degPath_(degPath) {}

double AngleView::Rad() const {
  if (radPath_ != nullptr) {
    return properties_.Get(radPath_);
  }

  return DegToRadValue(Deg());
}

double AngleView::Deg() const {
  if (degPath_ != nullptr) {
    return properties_.Get(degPath_);
  }

  return RadToDegValue(Rad());
}

MutableAngleView::MutableAngleView(FlightProperties &properties,
    const char *radPath, const char *degPath)
    : properties_(properties), radPath_(radPath), degPath_(degPath) {}

double MutableAngleView::Rad() const {
  if (radPath_ != nullptr) {
    return properties_.Get(radPath_);
  }

  return DegToRadValue(Deg());
}

double MutableAngleView::Deg() const {
  if (degPath_ != nullptr) {
    return properties_.Get(degPath_);
  }

  return RadToDegValue(Rad());
}

void MutableAngleView::SetRad(double value) const {
  if (radPath_ != nullptr) {
    properties_.Set(radPath_, value);
    return;
  }

  if (degPath_ != nullptr) {
    properties_.Set(degPath_, RadToDegValue(value));
  }
}

void MutableAngleView::SetDeg(double value) const {
  if (degPath_ != nullptr) {
    properties_.Set(degPath_, value);
    return;
  }

  if (radPath_ != nullptr) {
    properties_.Set(radPath_, DegToRadValue(value));
  }
}

AngularRateView::AngularRateView(const FlightProperties &properties,
    const char *rateRadPerSecPath, const char *dotRadPerSec2Path)
    : properties_(properties), rateRadPerSecPath_(rateRadPerSecPath),
      dotRadPerSec2Path_(dotRadPerSec2Path) {}

double AngularRateView::RadPerSec() const {
  return properties_.Get(rateRadPerSecPath_);
}

double AngularRateView::DegPerSec() const {
  return RadToDegValue(RadPerSec());
}

double AngularRateView::DotRadPerSec2() const {
  return properties_.Get(dotRadPerSec2Path_);
}

double AngularRateView::DotDegPerSec2() const {
  return RadToDegValue(DotRadPerSec2());
}

LinearVelocityView::LinearVelocityView(const FlightProperties &properties,
    const char *velocityFpsPath, const char *dotFps2Path)
    : properties_(properties), velocityFpsPath_(velocityFpsPath),
      dotFps2Path_(dotFps2Path) {}

double LinearVelocityView::Mps() const {
  return FeetPerSecToMetersPerSec(properties_.Get(velocityFpsPath_));
}

double LinearVelocityView::DotMps2() const {
  return FeetPerSec2ToMetersPerSec2(properties_.Get(dotFps2Path_));
}

SpeedView::SpeedView(const FlightProperties &properties, const char *fpsPath,
    const char *ktsPath)
    : properties_(properties), fpsPath_(fpsPath), ktsPath_(ktsPath) {}

double SpeedView::Mps() const { return FeetPerSecToMetersPerSec(Fps()); }

double SpeedView::Kts() const {
  if (ktsPath_ != nullptr) {
    return properties_.Get(ktsPath_);
  }

  return FeetPerSecToKts(Fps());
}

double SpeedView::Fps() const {
  if (fpsPath_ != nullptr) {
    return properties_.Get(fpsPath_);
  }

  return KtsToFeetPerSec(Kts());
}

double SpeedView::FtPerMin() const { return Fps() * 60.0; }

MutableSpeedView::MutableSpeedView(FlightProperties &properties,
    const char *fpsPath, const char *ktsPath)
    : properties_(properties), fpsPath_(fpsPath), ktsPath_(ktsPath) {}

double MutableSpeedView::Mps() const {
  return FeetPerSecToMetersPerSec(Fps());
}

double MutableSpeedView::Kts() const {
  if (ktsPath_ != nullptr) {
    return properties_.Get(ktsPath_);
  }

  return FeetPerSecToKts(Fps());
}

double MutableSpeedView::Fps() const {
  if (fpsPath_ != nullptr) {
    return properties_.Get(fpsPath_);
  }

  return KtsToFeetPerSec(Kts());
}

double MutableSpeedView::FtPerMin() const { return Fps() * 60.0; }

void MutableSpeedView::SetKts(double value) const {
  if (ktsPath_ != nullptr) {
    properties_.Set(ktsPath_, value);
    return;
  }

  if (fpsPath_ != nullptr) {
    properties_.Set(fpsPath_, KtsToFeetPerSec(value));
  }
}

FlightProperties::FlightProperties(FGFDMExec &fdmExec)
    : fdmExec_(fdmExec) {}

double FlightProperties::Get(const std::string &name) const {
  return fdmExec_.GetPropertyValue(name);
}

void FlightProperties::Set(const std::string &name, double value) {
  fdmExec_.SetPropertyValue(name, value);
}

MutableTimeView FlightProperties::SimTime() {
  return MutableTimeView(*this, SimTimeSec);
}

TimeView FlightProperties::SimTime() const {
  return TimeView(*this, SimTimeSec);
}

MutableDistanceView FlightProperties::AltitudeAgl() {
  return MutableDistanceView(*this, AltitudeAglFt);
}

DistanceView FlightProperties::AltitudeAgl() const {
  return DistanceView(*this, AltitudeAglFt);
}

MutableSpeedView FlightProperties::CalibratedAirspeed() {
  return MutableSpeedView(*this, nullptr, CalibratedAirspeedKts);
}

SpeedView FlightProperties::CalibratedAirspeed() const {
  return SpeedView(*this, nullptr, CalibratedAirspeedKts);
}

SpeedView FlightProperties::TrueAirspeed() const {
  return SpeedView(*this, TrueAirspeedFps, TrueAirspeedKts);
}

SpeedView FlightProperties::VerticalSpeed() const {
  return SpeedView(*this, VerticalSpeedFps, nullptr);
}

LinearVelocityView FlightProperties::U() const {
  return LinearVelocityView(*this, UFps, UDotFtPerSec2);
}

LinearVelocityView FlightProperties::V() const {
  return LinearVelocityView(*this, VFps, VDotFtPerSec2);
}

LinearVelocityView FlightProperties::W() const {
  return LinearVelocityView(*this, WFps, WDotFtPerSec2);
}

MutableAngleView FlightProperties::Roll() {
  return MutableAngleView(*this, RollRad, nullptr);
}

AngleView FlightProperties::Roll() const {
  return AngleView(*this, RollRad, nullptr);
}

MutableAngleView FlightProperties::Pitch() {
  return MutableAngleView(*this, PitchRad, nullptr);
}

AngleView FlightProperties::Pitch() const {
  return AngleView(*this, PitchRad, nullptr);
}

AngleView FlightProperties::Alpha() const {
  return AngleView(*this, nullptr, AlphaDeg);
}

AngleView FlightProperties::Beta() const {
  return AngleView(*this, nullptr, BetaDeg);
}

AngularRateView FlightProperties::P() const {
  return AngularRateView(*this, RollRateRadPerSec, PdotRadPerSec2);
}

AngularRateView FlightProperties::Q() const {
  return AngularRateView(*this, PitchRateRadPerSec, QdotRadPerSec2);
}

AngularRateView FlightProperties::R() const {
  return AngularRateView(*this, YawRateRadPerSec, RdotRadPerSec2);
}
} // namespace JSBSim
