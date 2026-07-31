#pragma once

#include "application/sim/gnc/TrimTypes.hpp"
#include "flightui/core/UICommon.hpp"

namespace gui {
struct TrimPanelProps {
  gnc::TrimRequest &request;
  const gnc::TrimResult &result;
  bool hasResult;
  bool &resultOpen;
  bool &residualOpen;
  bool canRequestTrim;
  FlightUI::Action requestRunICTrim;
  FlightUI::Action requestCurrentStateTrim;
};

class TrimPanel {
public:
  static void Draw(TrimPanelProps props);
};
} // namespace gui
