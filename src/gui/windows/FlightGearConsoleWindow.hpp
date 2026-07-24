#pragma once

#include "gui/Window.hpp"
class FlightGearConsoleWindow final : public gui::Window {
public:
  FlightGearConsoleWindow();

protected:
  void OnUpdate(gui::GUI &gui) override;
};
