#pragma once

#include "flightui/core/UIElement.hpp"

#include <memory>

namespace FlightUI {
class VerticalLayoutBuilder {
public:
  VerticalLayoutBuilder();
  explicit VerticalLayoutBuilder(Children children);
  VerticalLayoutBuilder(const VerticalLayoutBuilder &other);
  VerticalLayoutBuilder(VerticalLayoutBuilder &&other) noexcept;
  VerticalLayoutBuilder &operator=(const VerticalLayoutBuilder &other);
  VerticalLayoutBuilder &operator=(VerticalLayoutBuilder &&other) noexcept;
  ~VerticalLayoutBuilder();

  VerticalLayoutBuilder &SetSpacing(float spacing);
  VerticalLayoutBuilder &Spacing(float spacing);
  VerticalLayoutBuilder operator+(UIElement child) const;

  UIElement operator[](UIElement child) const;
  UIElement operator[](Children children) const;
  UIElement operator[](ChildrenBuilder children) const;

  operator UIElement() const;

private:
  class Impl;
  std::unique_ptr<Impl> m_Impl;
};

VerticalLayoutBuilder VerticalLayout();
VerticalLayoutBuilder VerticalLayout(Children children);
} // namespace FlightUI
