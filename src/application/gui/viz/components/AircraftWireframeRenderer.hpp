#pragma once

#include "application/gui/viz/core/Component.hpp"
#include "application/gui/viz/core/Math.hpp"

#include <vector>

namespace viz {
class AircraftWireframeRenderer final : public Component {
public:
  AircraftWireframeRenderer();

  void OnTick(const TickContext &context) override;
  void Render(RenderContext &context) const override;

private:
  struct Segment {
    Vec3 a;
    Vec3 b;
  };

  std::vector<Segment> airframeSegments_;
};
} // namespace viz
