#pragma once

#include "gui/Window.hpp"

namespace gui {
class SamplePlotWindow final : public Window {
public:
  SamplePlotWindow();

protected:
  void OnUpdate(GUI &gui) override;
};
} // namespace gui
