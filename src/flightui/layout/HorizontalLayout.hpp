#pragma once

#include "flightui/core/UIElement.hpp"

#include <memory>

namespace FlightUI {
class HorizontalLayoutBuilder {
public:
  HorizontalLayoutBuilder();
  explicit HorizontalLayoutBuilder(Children children);
  HorizontalLayoutBuilder(const HorizontalLayoutBuilder &other);
  HorizontalLayoutBuilder(HorizontalLayoutBuilder &&other) noexcept;
  HorizontalLayoutBuilder &operator=(const HorizontalLayoutBuilder &other);
  HorizontalLayoutBuilder &operator=(HorizontalLayoutBuilder &&other) noexcept;
  ~HorizontalLayoutBuilder();

  HorizontalLayoutBuilder &SetSpacing(float spacing);
  HorizontalLayoutBuilder &Spacing(float spacing);
  HorizontalLayoutBuilder operator+(UIElement child) const;

  UIElement operator[](UIElement child) const;
  UIElement operator[](Children children) const;
  UIElement operator[](ChildrenBuilder children) const;

  operator UIElement() const;

private:
  class Impl;
  std::unique_ptr<Impl> m_Impl;
};

HorizontalLayoutBuilder HorizontalLayout();
HorizontalLayoutBuilder HorizontalLayout(Children children);
} // namespace FlightUI
