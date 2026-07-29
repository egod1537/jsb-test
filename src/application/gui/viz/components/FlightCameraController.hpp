#pragma once

#include "application/gui/viz/core/Component.hpp"

namespace viz {
class CameraComponent;

class FlightCameraController final : public Component {
public:
  void SetCamera(CameraComponent *camera) { camera_ = camera; }
  void Update(const UpdateContext &context) override;

private:
  void ApplyOrbitCamera(const UpdateContext &context) const;
  void ApplyThirdPersonCamera(const UpdateContext &context) const;

  CameraComponent *camera_ = nullptr;
};
} // namespace viz
