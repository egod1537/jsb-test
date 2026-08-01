#pragma once

#include "application/gui/Window.hpp"

namespace gui {
class FlightVizWindow final : public gui::Window {
public:
  FlightVizWindow();

protected:
  void OnRender(gui::GUI &gui) override;
};
} // namespace gui
