#include "application/gui/viz/components/AircraftWireframeRenderer.hpp"

#include "application/gui/viz/core/Transform.hpp"
#include "application/gui/viz/render/LineCanvas.hpp"

namespace viz {
AircraftWireframeRenderer::AircraftWireframeRenderer()
    : airframeSegments_{
          {{1.7F, 0.0F, 0.0F}, {-1.45F, 0.0F, 0.0F}},
          {{0.15F, -2.25F, 0.0F}, {0.15F, 2.25F, 0.0F}},
          {{-1.35F, -0.75F, 0.0F}, {-1.35F, 0.75F, 0.0F}},
          {{-1.45F, 0.0F, 0.0F}, {-1.35F, 0.0F, 0.75F}},
          {{-1.35F, 0.0F, 0.75F}, {-1.15F, 0.0F, 0.0F}},
          {{1.7F, 0.0F, 0.0F}, {0.15F, -2.25F, 0.0F}},
          {{1.7F, 0.0F, 0.0F}, {0.15F, 2.25F, 0.0F}},
          {{-1.45F, 0.0F, 0.0F}, {0.15F, -2.25F, 0.0F}},
          {{-1.45F, 0.0F, 0.0F}, {0.15F, 2.25F, 0.0F}},
          {{-1.45F, 0.0F, 0.0F}, {-1.35F, -0.75F, 0.0F}},
          {{-1.45F, 0.0F, 0.0F}, {-1.35F, 0.75F, 0.0F}},
          {{-1.35F, -0.75F, 0.0F}, {-1.35F, 0.75F, 0.0F}},
      } {}

void AircraftWireframeRenderer::OnTick(const TickContext &context) {
  const auto &state = context.snapshot.aircraftState;
  Transform &transform = GetTransform();
  transform.SetPosition({0.0F, 0.0F, 0.35F});
  transform.SetRotationDeg({
      -static_cast<float>(state.rollDeg),
      -static_cast<float>(state.pitchDeg),
      static_cast<float>(state.headingDeg),
  });
}

void AircraftWireframeRenderer::Render(RenderContext &context) const {
  const Transform &transform = GetTransform();
  const ImU32 bodyColor = IM_COL32(238, 242, 248, 255);

  for (const Segment &segment : airframeSegments_) {
    context.canvas.Line(transform.TransformPoint(segment.a),
        transform.TransformPoint(segment.b),
        bodyColor,
        2.0F);
  }

  const Vec3 origin = transform.TransformPoint({0.0F, 0.0F, 0.0F});
  context.canvas.Line(origin,
      transform.TransformPoint({2.3F, 0.0F, 0.0F}),
      IM_COL32(248, 92, 92, 255),
      2.0F);
  context.canvas.Line(origin,
      transform.TransformPoint({0.0F, 1.35F, 0.0F}),
      IM_COL32(92, 210, 132, 255),
      2.0F);
  context.canvas.Line(origin,
      transform.TransformPoint({0.0F, 0.0F, 1.25F}),
      IM_COL32(112, 168, 255, 255),
      2.0F);
}
} // namespace viz
