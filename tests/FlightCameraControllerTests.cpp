#include "application/gui/viz/components/FlightCameraController.hpp"

#include "application/gui/viz/render/CameraComponent.hpp"

#include <cassert>

namespace {
constexpr viz::Vec3 AircraftPosition{0.0F, 0.0F, 0.35F};

void RequireAircraftTracked(float visualAltitude) {
  viz::CameraComponent camera;
  viz::FlightCameraController controller;
  controller.SetCamera(&camera);

  viz::FrameSnapshot snapshot{};
  snapshot.viewMode = viz::ViewMode::ThirdPerson;
  snapshot.visualAltitude = visualAltitude;
  snapshot.aircraftState.headingDeg = 37.0;
  snapshot.aircraftState.pitchDeg = 5.0;
  controller.OnTick(viz::TickContext{snapshot});

  const viz::CameraView view = camera.BuildView();
  const viz::Vec3 directionToAircraft =
      viz::Normalize(AircraftPosition - view.eye);

  assert(viz::Dot(view.forward, directionToAircraft) > 0.98F);
}
} // namespace

int main() {
  RequireAircraftTracked(0.35F);
  RequireAircraftTracked(52.0F);
  return 0;
}
