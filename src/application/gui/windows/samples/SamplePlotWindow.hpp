#pragma once

#include "application/gui/Window.hpp"

namespace gui {
class SamplePlotWindow final : public Window {
public:
  SamplePlotWindow();

protected:
  void OnRender(GUI &gui) override;
};
} // namespace gui
