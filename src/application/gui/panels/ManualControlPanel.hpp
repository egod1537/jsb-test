#pragma once

#include "application/gui/panels/AutopilotPanel.hpp"

namespace sim {
class Aircraft;
}

namespace gui {
class ManualControlPanel {
public:
  static void Draw(sim::Aircraft &aircraft,
      const AutopilotPanelState &autopilotState);
};
} // namespace gui
