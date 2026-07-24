#pragma once

#include "gui/Window.hpp"

namespace gui {
class FlightControlWindow final : public Window {
public:
  FlightControlWindow();

protected:
  void OnUpdate(GUI &gui) override;
};
} // namespace gui
