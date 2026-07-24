#include "flightui/controls/Text.hpp"

#include "flightui/core/UIElementFactory.hpp"

#include <imgui.h>

#include <utility>

namespace FlightUI {
UIElement Text(std::string text) {
  return CreateElement(
      [text = std::move(text)] { ImGui::TextUnformatted(text.c_str()); });
}
} // namespace FlightUI
