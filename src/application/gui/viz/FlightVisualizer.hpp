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
  FlightVisualizer();
  ~FlightVisualizer();

  bool Update(const sim::Aircraft &aircraft);
  void RenderScene();
  void RenderAircraftWireframe();

private:
  void BuildScene();
  void HandleInput();
  void RenderViewOptionsMenu(ImVec2 min, ImVec2 max);
  void ToggleViewMode();
  void UpdateGroundScroll(const sim::AircraftState &state);

  Scene scene_;
  CameraComponent *mainCamera_ = nullptr;
  FrameSnapshot snapshot_{};
  ViewMode viewMode_ = ViewMode::Orbit;
  ViewOptions viewOptions_{};
  Vec3 groundScroll_{};
  double lastMotionSampleTimeSec_ = 0.0;
  bool hasMotionSample_ = false;
};
} // namespace viz
