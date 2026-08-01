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
  void OnRender(gui::GUI &) override;

private:
  // Plot construction
  FlightUI::UIElement DrawAerodynamicAnglesPlot() const;
  FlightUI::UIElement DrawBodyRatesPlot() const;
  FlightUI::UIElement DrawBodyAccelerationsPlot() const;
  FlightUI::UIElement DrawAngularAccelerationsPlot() const;

  // Window rendering and sampling
  void DrawCurrentValues(const sim::AircraftState &state) const;
  void DrawPlotGrid() const;
  void DrawWindow(sim::Simulation &sim) const;
  void OnRecordSamples(sim::Simulation &sim);

  // Sample timeline
  ds::RingBuffer<double> timeHistory_;

  // Aerodynamic angles
  ds::RingBuffer<double> alphaDegHistory_;
  ds::RingBuffer<double> betaDegHistory_;

  // Body rates
  ds::RingBuffer<double> pDegPerSecHistory_;
  ds::RingBuffer<double> qDegPerSecHistory_;
  ds::RingBuffer<double> rDegPerSecHistory_;

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
