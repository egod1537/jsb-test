#pragma once

#include "application/gui/Window.hpp"

namespace gnc {
struct LinearizationResult;
}

namespace gui {
class LinearizationWindow final : public Window {
public:
  LinearizationWindow();

protected:
  void OnRender(GUI &gui) override;

private:
  // Matrix rendering
  void DrawResult(const gnc::LinearizationResult &result) const;
};
} // namespace gui
