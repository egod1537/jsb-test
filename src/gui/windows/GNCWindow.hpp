#pragma once

#include "gnc/TrimTypes.hpp"
#include "gui/Window.hpp"

namespace gui {
class GNCWindow final : public gui::Window {
public:
  GNCWindow();

protected:
  void OnUpdate(gui::GUI &gui) override;

private:
  gnc::TrimRequest trimRequest_;
  gnc::TrimResult trimResult_;
  bool trimHasResult_ = false;
  bool trimResultOpen_ = true;
  bool trimResidualOpen_ = true;
};
} // namespace gui
