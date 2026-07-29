#include "gui/Window.hpp"
#include <utility>

namespace gui {
Window::Window(std::string title) : title_(std::move(title)) {}

Window::~Window() = default;

void Window::Update(GUI &gui) {
  if (!visible_) {
    return;
  }

  if (ImGui::Begin(title_.c_str(), &visible_, GetWindowFlags())) {
    OnUpdate(gui);
  }
  ImGui::End();
}

ImGuiWindowFlags Window::GetWindowFlags() const {
  return ImGuiWindowFlags_None;
}
} // namespace gui
