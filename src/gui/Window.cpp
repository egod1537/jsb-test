#include "gui/Window.hpp"
#include <utility>

namespace gui {
Window::Window(std::string title) : title_(std::move(title)) {}

Window::~Window() = default;

void Window::Update(GUI &gui) {
  if (!open_) {
    return;
  }

  bool isOpen = open_;
  if (ImGui::Begin(title_.c_str(), &isOpen, GetWindowFlags())) {
    OnUpdate(gui);
  }
  ImGui::End();

  open_ = isOpen;
}

ImGuiWindowFlags Window::GetWindowFlags() const {
  return ImGuiWindowFlags_None;
}
} // namespace gui
