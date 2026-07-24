#include "flightui/core/UICommon.hpp"
#include "flightui/core/UIRenderHelpers.hpp"

#include <imgui.h>

namespace FlightUI::Internal {
IdScope::IdScope(const std::string &id) : m_Active(!id.empty()) {
  if (m_Active) {
    ImGui::PushID(id.c_str());
  }
}

IdScope::~IdScope() {
  if (m_Active) {
    ImGui::PopID();
  }
}

DisabledScope::DisabledScope(bool disabled) : m_Active(disabled) {
  if (m_Active) {
    ImGui::BeginDisabled();
  }
}

DisabledScope::~DisabledScope() {
  if (m_Active) {
    ImGui::EndDisabled();
  }
}

ItemWidthScope::ItemWidthScope(float width) : m_Active(width > 0.0F) {
  if (m_Active) {
    ImGui::PushItemWidth(width);
  }
}

ItemWidthScope::~ItemWidthScope() {
  if (m_Active) {
    ImGui::PopItemWidth();
  }
}

void ShowTooltipIfHovered(const std::string &tooltip) {
  if (tooltip.empty() || !ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal)) {
    return;
  }

  ImGui::SetTooltip("%s", tooltip.c_str());
}
} // namespace FlightUI::Internal
