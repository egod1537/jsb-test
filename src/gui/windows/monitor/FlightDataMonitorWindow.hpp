#pragma once

#include "flightui/core/UIElement.hpp"
#include "flightui/plot/RingBuffer.hpp"
#include "gui/Window.hpp"
#include "simulation/AircraftState.hpp"

namespace sim {
class Simulation;
}

namespace gui {

class FlightDataMonitorWindow final : public gui::Window {
public:
  FlightDataMonitorWindow();

protected:
  void OnUpdate(gui::GUI &) override;

private:
  FlightUI::UIElement DrawAerodynamicAnglesPlot() const;
  FlightUI::UIElement DrawBodyRatesPlot() const;
  FlightUI::UIElement DrawBodyAccelerationsPlot() const;
  FlightUI::UIElement DrawAngularAccelerationsPlot() const;
  void DrawCurrentValues(const sim::AircraftState &state) const;
  void DrawWindow(sim::Simulation &sim) const;
  void OnRecordSamples(sim::Simulation &sim);

  ds::RingBuffer<double> timeHistory_;
  ds::RingBuffer<double> alphaDegHistory_;
  ds::RingBuffer<double> betaDegHistory_;
  ds::RingBuffer<double> pDegPerSecHistory_;
  ds::RingBuffer<double> qDegPerSecHistory_;
  ds::RingBuffer<double> rDegPerSecHistory_;
  ds::RingBuffer<double> uDotMps2History_;
  ds::RingBuffer<double> vDotMps2History_;
  ds::RingBuffer<double> wDotMps2History_;
  ds::RingBuffer<double> pDotDegPerSec2History_;
  ds::RingBuffer<double> qDotDegPerSec2History_;
  ds::RingBuffer<double> rDotDegPerSec2History_;
};

} // namespace gui
