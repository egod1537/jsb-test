#pragma once

#include "application/gui/Window.hpp"
class FlightGearConsoleWindow final : public gui::Window {
public:
  FlightGearConsoleWindow();

protected:
  void OnRender(gui::GUI &gui) override;
};
