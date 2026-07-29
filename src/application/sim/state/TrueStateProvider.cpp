#include "application/sim/state/TrueStateProvider.hpp"

#include "application/sim/Aircraft.hpp"

namespace {
constexpr double FeetToMeters = 0.3048;
constexpr double DegToRad = 0.017453292519943295769;

double DegToRadValue(double value) { return value * DegToRad; }
} // namespace

namespace state {
TrueStateProvider::TrueStateProvider(const sim::Aircraft &aircraft)
    : aircraft_(aircraft) {}

AircraftState TrueStateProvider::GetState() const {
  const sim::AircraftState aircraftState = aircraft_.GetAircraftState();
  const sim::InitialCondition currentCondition =
      aircraft_.CaptureCurrentCondition();

  AircraftState state{};
  state.simTimeSec = aircraftState.simulationTimeSec;
  state.latitudeRad = DegToRadValue(currentCondition.latitudeDeg);
  state.longitudeRad = DegToRadValue(currentCondition.longitudeDeg);
  state.altitudeM = currentCondition.altitudeFt * FeetToMeters;
  state.rollRad = DegToRadValue(currentCondition.rollDeg);
  state.pitchRad = DegToRadValue(currentCondition.pitchDeg);
  state.headingRad = DegToRadValue(currentCondition.headingDeg);
  state.courseRad = state.headingRad;
  state.uMps = aircraftState.uMps;
  state.vMps = aircraftState.vMps;
  state.wMps = aircraftState.wMps;
  state.pRadPerSec = currentCondition.pRadPerSec;
  state.qRadPerSec = currentCondition.qRadPerSec;
  state.rRadPerSec = currentCondition.rRadPerSec;
  state.airspeedMps = aircraftState.trueAirspeedMps;

  return state;
}
} // namespace state
