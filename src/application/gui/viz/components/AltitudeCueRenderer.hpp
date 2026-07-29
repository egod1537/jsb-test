#pragma once

#include "application/gui/viz/core/Component.hpp"

namespace viz {
class AltitudeCueRenderer final : public Component {
public:
  void Render(RenderContext &context) const override;
};
} // namespace viz
