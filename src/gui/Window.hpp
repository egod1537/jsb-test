#pragma once

#include "gui/Component.hpp"
#include "imgui.h"
#include <string>

namespace gui {
class Window : public Component {
public:
  explicit Window(std::string title);
  ~Window() override;

  const std::string &GetTitle() const { return title_; }

  bool IsOpen() const { return open_; }
  void SetOpen(bool open) { open_ = open; }

protected:
  void Update(GUI &gui) final;

  virtual ImGuiWindowFlags GetWindowFlags() const;
  virtual void OnUpdate(GUI &gui) = 0;

private:
  std::string title_;
  bool open_ = true;
};
} // namespace gui
