#pragma once

#include "flightui/core/UIElement.hpp"

#include <memory>
#include <string>

namespace FlightUI {
class ButtonBuilder {
public:
  explicit ButtonBuilder(std::string label, Action onClick = {});
  ButtonBuilder(const ButtonBuilder &other);
  ButtonBuilder(ButtonBuilder &&other) noexcept;
  ButtonBuilder &operator=(const ButtonBuilder &other);
  ButtonBuilder &operator=(ButtonBuilder &&other) noexcept;
  ~ButtonBuilder();

  ButtonBuilder &SetOnAction(Action onClick);
  ButtonBuilder &SetSize(Vector2 size);
  ButtonBuilder &SetWidth(float width);
  ButtonBuilder &SetHeight(float height);
  ButtonBuilder &SetEnabled(bool enabled);
  ButtonBuilder &SetTooltip(std::string tooltip);
  ButtonBuilder &SetId(std::string id);

  ButtonBuilder &OnAction(Action onClick);
  ButtonBuilder &Size(Vector2 size);
  ButtonBuilder &Width(float width);
  ButtonBuilder &Widht(float width);
  ButtonBuilder &Height(float height);
  ButtonBuilder &Enabled(bool enabled);
  ButtonBuilder &Tooltip(std::string tooltip);
  ButtonBuilder &Id(std::string id);

  operator UIElement() const;

private:
  class Impl;
  std::unique_ptr<Impl> m_Impl;
};

ButtonBuilder Button(std::string label, Action onClick = {});
} // namespace FlightUI
