#pragma once

#include "application/gui/viz/core/FrameSnapshot.hpp"
#include "application/gui/viz/core/Scene.hpp"

struct ImVec2;

namespace sim {
class Aircraft;
}

namespace viz {
class CameraComponent;

class FlightVisualizer {
public:
  // Lifetime and frame update
  FlightVisualizer();
  ~FlightVisualizer();

  bool Tick(const sim::Aircraft &aircraft);

  // Rendering
  void RenderScene();
  void RenderAircraftWireframe();

private:
  // Scene setup and interaction
  void BuildScene();
  void HandleInput();
  void RenderViewOptionsMenu(ImVec2 min, ImVec2 max);
  void ToggleViewMode();

  // Aircraft synchronization
  void SyncGroundScroll(const sim::AircraftState &state);

  // Scene state
  Scene scene_;
  CameraComponent *mainCamera_ = nullptr;
  FrameSnapshot snapshot_{};

  // View state
  ViewMode viewMode_ = ViewMode::Orbit;
  ViewOptions viewOptions_{};
  Vec3 groundScroll_{};

  // Motion cache
  double lastMotionSampleTimeSec_ = 0.0;
  bool hasMotionSample_ = false;
};
} // namespace viz
