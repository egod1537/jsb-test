#include "application/gui/viz/components/FlightCameraController.hpp"

#include "application/gui/viz/render/CameraComponent.hpp"

#include <algorithm>

namespace {
viz::Vec3 AircraftForward(const sim::AircraftState &state) {
  viz::Vec3 forward{1.0F, 0.0F, 0.0F};
  forward = viz::RotateY(forward, -static_cast<float>(state.pitchDeg) *
                                      viz::DegToRad);
  forward =
      viz::RotateZ(forward, static_cast<float>(state.headingDeg) * viz::DegToRad);
  return viz::Normalize(forward);
}
} // namespace

namespace viz {
void FlightCameraController::OnTick(const TickContext &context) {
  if (camera_ == nullptr) {
    return;
  }

  switch (context.snapshot.viewMode) {
  case ViewMode::ThirdPerson:
    ApplyThirdPersonCamera(context);
    break;
  case ViewMode::Orbit:
  default:
    ApplyOrbitCamera(context);
    break;
  }
}

void FlightCameraController::ApplyOrbitCamera(
    const TickContext &context) const {
  const float altitude = std::max(context.snapshot.visualAltitude, 0.35F);
  const float pullBack = std::min(altitude * 0.18F, 8.0F);
  const float lift = std::min(altitude * 0.10F, 8.0F);
  const float lookDown = std::min(altitude * 0.36F, 18.0F);

  camera_->SetEye({5.5F + pullBack * 0.35F, -8.0F - pullBack, 4.2F + lift});
  camera_->SetTarget({0.0F, 0.0F, 0.35F - lookDown});
  camera_->SetWorldUp({0.0F, 0.0F, 1.0F});
}

void FlightCameraController::ApplyThirdPersonCamera(
    const TickContext &context) const {
  const Vec3 aircraftPosition{0.0F, 0.0F, 0.35F};
  const Vec3 forward = AircraftForward(context.snapshot.aircraftState);
  const float altitude = std::max(context.snapshot.visualAltitude, 0.35F);
  const float chaseDistance = 7.0F + std::min(altitude * 0.22F, 9.0F);
  const float lookDown = std::min(altitude * 0.72F, 32.0F);
  const Vec3 eye =
      aircraftPosition - forward * chaseDistance
      + Vec3{0.0F, 0.0F, 2.4F + std::min(altitude * 0.06F, 4.0F)};
  const Vec3 target =
      aircraftPosition + forward * 4.0F + Vec3{0.0F, 0.0F, -lookDown};

  camera_->SetEye(eye);
  camera_->SetTarget(target);
  camera_->SetWorldUp({0.0F, 0.0F, 1.0F});
}
} // namespace viz
