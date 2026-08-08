#pragma once

#include "application/sim/Component.hpp"
#include "application/sim/control/ControlInput.hpp"
#include "application/sim/control/FlightControlMode.hpp"
#include "application/sim/control/ManualFlightControlController.hpp"
#include "application/sim/gnc/Autopilot.hpp"

#include <optional>

namespace sim {
struct Tick;
} // namespace sim

namespace control {
class FlightControlManager final : public sim::Component {
public:
  FlightControlManager();

  // Active source
  FlightControlMode GetMode() const;
  void SetMode(FlightControlMode mode);

  // Owned controllers
  ManualFlightControlController &GetManualController();
  const ManualFlightControlController &GetManualController() const;
  gnc::Autopilot &GetAutopilot();
  const gnc::Autopilot &GetAutopilot() const;

  // Controller state
  void ResetControllers();
  void SynchronizeWithTrimResult(const gnc::TrimResult &trimResult);

protected:
  bool OnTick(const sim::Tick &tick) override;

private:
  // Control routing
  std::optional<ControlInput> ProduceControlInput(sim::Aircraft &aircraft,
      const sim::Tick &tick);

  // Control sources
  ManualFlightControlController manualController_;
  gnc::Autopilot autopilot_;

  // Routing state
  FlightControlMode mode_ = FlightControlMode::Manual;
};
} // namespace control
