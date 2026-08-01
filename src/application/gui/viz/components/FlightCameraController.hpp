#pragma once

#include "application/gui/viz/core/Component.hpp"

namespace viz {
class CameraComponent;

class FlightCameraController final : public Component {
public:
  void SetCamera(CameraComponent *camera) { camera_ = camera; }
  void OnTick(const TickContext &context) override;

private:
  void ApplyOrbitCamera(const TickContext &context) const;
  void ApplyThirdPersonCamera(const TickContext &context) const;

  CameraComponent *camera_ = nullptr;
};
} // namespace viz
