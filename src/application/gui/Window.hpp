#pragma once

#include "application/gui/Component.hpp"
#include "imgui.h"
#include <string>

namespace gui {
class Window : public Component {
public:
  explicit Window(std::string title);
  ~Window() override;

  const std::string &GetTitle() const { return title_; }

  bool IsVisible() const { return visible_; }
  void SetVisible(bool visible) { visible_ = visible; }
  bool *GetVisiblePtr() { return &visible_; }

protected:
  void OnTick(GUI &gui) final;

  virtual ImGuiWindowFlags GetWindowFlags() const;
  virtual void OnRender(GUI &gui) = 0;

private:
  std::string title_;
  bool visible_ = true;
};
} // namespace gui
