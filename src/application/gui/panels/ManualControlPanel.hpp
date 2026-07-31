#pragma once

#include "application/gui/panels/AutopilotPanel.hpp"

namespace control {
class ManualFlightControlController;
}

namespace sim {
class Aircraft;
}

namespace gui {
class ManualControlPanel {
public:
  static void Draw(control::ManualFlightControlController &manualController,
      const sim::Aircraft &aircraft,
      const AutopilotPanelState &autopilotState);
};
} // namespace gui
