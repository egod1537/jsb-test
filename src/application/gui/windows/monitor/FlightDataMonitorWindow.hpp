#pragma once

#include "flightui/core/UIElement.hpp"
#include "flightui/plot/RingBuffer.hpp"
#include "application/gui/Window.hpp"
#include "application/sim/AircraftState.hpp"

namespace sim {
class Simulation;
}

namespace gui {

class FlightDataMonitorWindow final : public gui::Window {
public:
  FlightDataMonitorWindow();

protected:
  void OnRender(gui::GUI &gui) override;

private:
  struct PlotVisibility {
    bool aerodynamicAngles = true;
    bool attitude = false;
    bool bodyVelocities = false;
    bool bodyRates = true;
    bool airspeed = false;
    bool altitude = false;
    bool bodyAccelerations = true;
    bool angularAccelerations = true;
  };

  // Plot construction
  FlightUI::UIElement DrawAerodynamicAnglesPlot() const;
  FlightUI::UIElement DrawAttitudePlot() const;
  FlightUI::UIElement DrawBodyVelocitiesPlot() const;
  FlightUI::UIElement DrawBodyRatesPlot() const;
  FlightUI::UIElement DrawAirspeedPlot() const;
  FlightUI::UIElement DrawAltitudePlot() const;
  FlightUI::UIElement DrawBodyAccelerationsPlot() const;
  FlightUI::UIElement DrawAngularAccelerationsPlot() const;

  // Window rendering and sampling
  void DrawCurrentValues(const sim::AircraftState &state) const;
  void DrawPlotSelector();
  void DrawPlotGrid() const;
  void DrawWindow(sim::Simulation &sim);
  void OnRecordSamples(sim::Simulation &sim);

  // Plot display settings
  PlotVisibility plotVisibility_;

  // Sample timeline
  ds::RingBuffer<double> timeHistory_;

  // Aerodynamic angles
  ds::RingBuffer<double> alphaDegHistory_;
  ds::RingBuffer<double> betaDegHistory_;

  // Attitude
  ds::RingBuffer<double> rollDegHistory_;
  ds::RingBuffer<double> pitchDegHistory_;
  ds::RingBuffer<double> headingDegHistory_;

  // Body velocities
  ds::RingBuffer<double> uMpsHistory_;
  ds::RingBuffer<double> vMpsHistory_;
  ds::RingBuffer<double> wMpsHistory_;

  // Body rates
  ds::RingBuffer<double> pDegPerSecHistory_;
  ds::RingBuffer<double> qDegPerSecHistory_;
  ds::RingBuffer<double> rDegPerSecHistory_;

  // Air data
  ds::RingBuffer<double> calibratedAirspeedKtsHistory_;
  ds::RingBuffer<double> trueAirspeedKtsHistory_;
  ds::RingBuffer<double> altitudeAglFtHistory_;

  // Linear accelerations
  ds::RingBuffer<double> uDotMps2History_;
  ds::RingBuffer<double> vDotMps2History_;
  ds::RingBuffer<double> wDotMps2History_;

  // Angular accelerations
  ds::RingBuffer<double> pDotDegPerSec2History_;
  ds::RingBuffer<double> qDotDegPerSec2History_;
  ds::RingBuffer<double> rDotDegPerSec2History_;
};

} // namespace gui
