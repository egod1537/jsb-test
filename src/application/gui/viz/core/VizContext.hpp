#pragma once

#include "application/gui/viz/core/FrameSnapshot.hpp"

namespace viz {
class LineCanvas;

struct TickContext {
  const FrameSnapshot &snapshot;
};

struct RenderContext {
  const FrameSnapshot &snapshot;
  LineCanvas &canvas;
};
} // namespace viz
