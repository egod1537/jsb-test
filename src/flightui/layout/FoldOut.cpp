#include "flightui/layout/FoldOut.hpp"

#include "flightui/core/UIElementFactory.hpp"
#include "flightui/core/UIRenderHelpers.hpp"

#include <imgui.h>

#include <utility>

namespace FlightUI {
class FoldOutBuilder::Impl {
public:
  std::string Label;
  bool *Open = nullptr;
  bool DefaultOpen = false;
  ImGuiTreeNodeFlags Flags = ImGuiTreeNodeFlags_None;
  bool Enabled = true;
  bool Visible = true;
  std::string Tooltip;
  std::string Id;
};

namespace {
std::string MakeFoldOutLabel(const std::string &label, const std::string &id) {
  if (id.empty()) {
    return label;
  }

  return label + "###" + id;
}

bool ShouldTreePop(ImGuiTreeNodeFlags flags) {
  return (flags & ImGuiTreeNodeFlags_NoTreePushOnOpen) == 0;
}
} // namespace

FoldOutBuilder::FoldOutBuilder(std::string label)
    : m_Impl(std::make_unique<Impl>()) {
  m_Impl->Label = std::move(label);
}

FoldOutBuilder::FoldOutBuilder(const FoldOutBuilder &other)
    : m_Impl(other.m_Impl == nullptr ? nullptr
                                     : std::make_unique<Impl>(*other.m_Impl)) {}

FoldOutBuilder::FoldOutBuilder(FoldOutBuilder &&other) noexcept = default;

FoldOutBuilder &FoldOutBuilder::operator=(const FoldOutBuilder &other) {
  if (this != &other) {
    m_Impl = other.m_Impl == nullptr ? nullptr
                                     : std::make_unique<Impl>(*other.m_Impl);
  }

  return *this;
}

FoldOutBuilder &FoldOutBuilder::operator=(
    FoldOutBuilder &&other) noexcept = default;

FoldOutBuilder::~FoldOutBuilder() = default;

FoldOutBuilder &FoldOutBuilder::SetOpen(bool &isOpen) {
  m_Impl->Open = &isOpen;
  return *this;
}

FoldOutBuilder &FoldOutBuilder::SetDefaultOpen(bool open) {
  m_Impl->DefaultOpen = open;
  return *this;
}

FoldOutBuilder &FoldOutBuilder::SetFlags(ImGuiTreeNodeFlags flags) {
  m_Impl->Flags = flags;
  return *this;
}

FoldOutBuilder &FoldOutBuilder::SetFramed(bool enabled) {
  if (enabled) {
    m_Impl->Flags |= ImGuiTreeNodeFlags_Framed;
  } else {
    m_Impl->Flags &= ~ImGuiTreeNodeFlags_Framed;
  }

  return *this;
}

FoldOutBuilder &FoldOutBuilder::SetSpanAvailWidth(bool enabled) {
  if (enabled) {
    m_Impl->Flags |= ImGuiTreeNodeFlags_SpanAvailWidth;
  } else {
    m_Impl->Flags &= ~ImGuiTreeNodeFlags_SpanAvailWidth;
  }

  return *this;
}

FoldOutBuilder &FoldOutBuilder::SetEnabled(bool enabled) {
  m_Impl->Enabled = enabled;
  return *this;
}

FoldOutBuilder &FoldOutBuilder::SetVisible(bool visible) {
  m_Impl->Visible = visible;
  return *this;
}

FoldOutBuilder &FoldOutBuilder::SetTooltip(std::string tooltip) {
  m_Impl->Tooltip = std::move(tooltip);
  return *this;
}

FoldOutBuilder &FoldOutBuilder::SetId(std::string id) {
  m_Impl->Id = std::move(id);
  return *this;
}

FoldOutBuilder &FoldOutBuilder::Open(bool &isOpen) { return SetOpen(isOpen); }

FoldOutBuilder &FoldOutBuilder::DefaultOpen(bool open) {
  return SetDefaultOpen(open);
}

FoldOutBuilder &FoldOutBuilder::Flags(ImGuiTreeNodeFlags flags) {
  return SetFlags(flags);
}

FoldOutBuilder &FoldOutBuilder::Framed(bool enabled) {
  return SetFramed(enabled);
}

FoldOutBuilder &FoldOutBuilder::SpanAvailWidth(bool enabled) {
  return SetSpanAvailWidth(enabled);
}

FoldOutBuilder &FoldOutBuilder::Enabled(bool enabled) {
  return SetEnabled(enabled);
}

FoldOutBuilder &FoldOutBuilder::Visible(bool visible) {
  return SetVisible(visible);
}

FoldOutBuilder &FoldOutBuilder::Tooltip(std::string tooltip) {
  return SetTooltip(std::move(tooltip));
}

FoldOutBuilder &FoldOutBuilder::Id(std::string id) {
  return SetId(std::move(id));
}

UIElement FoldOutBuilder::operator[](UIElement child) const {
  Children children;
  children.push_back(std::move(child));
  return (*this)[std::move(children)];
}

UIElement FoldOutBuilder::operator[](ChildrenBuilder children) const {
  return (*this)[std::move(children).TakeChildren()];
}

UIElement FoldOutBuilder::operator[](Children children) const {
  Impl state = *m_Impl;
  return CreateElement([state, children = std::move(children)] {
    if (!state.Visible) {
      return;
    }

    if (state.Open != nullptr) {
      ImGui::SetNextItemOpen(*state.Open, ImGuiCond_Always);
    } else if (state.DefaultOpen) {
      ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    }

    const std::string label = MakeFoldOutLabel(state.Label, state.Id);
    Internal::DisabledScope disabledScope(!state.Enabled);
    const bool isOpen = ImGui::TreeNodeEx(label.c_str(), state.Flags);

    Internal::ShowTooltipIfHovered(state.Tooltip);

    if (isOpen) {
      for (const UIElement &childElement : children) {
        childElement.Render();
      }

      if (ShouldTreePop(state.Flags)) {
        ImGui::TreePop();
      }
    }

    if (state.Open != nullptr) {
      *state.Open = isOpen;
    }
  });
}

FoldOutBuilder FoldOut(std::string label) {
  return FoldOutBuilder(std::move(label));
}
} // namespace FlightUI
