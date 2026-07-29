#pragma once

#include "application/gui/Window.hpp"

namespace gui {
class FlightConsoleWindow final : public Window {
public:
  FlightConsoleWindow();

protected:
  void OnUpdate(GUI &gui) override;
};
} // namespace gui
