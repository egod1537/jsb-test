#pragma once

#include "flightui/core/UIElement.hpp"

#include <functional>
#include <memory>
#include <string>

namespace FlightUI {
using ToggleChangedAction = std::function<void(bool)>;

class ToggleBuilder {
public:
  ToggleBuilder(std::string label, bool value);
  ToggleBuilder(const ToggleBuilder &other);
  ToggleBuilder(ToggleBuilder &&other) noexcept;
  ToggleBuilder &operator=(const ToggleBuilder &other);
  ToggleBuilder &operator=(ToggleBuilder &&other) noexcept;
  ~ToggleBuilder();

  ToggleBuilder &SetOnChanged(ToggleChangedAction onChanged);
  ToggleBuilder &SetEnabled(bool enabled);
  ToggleBuilder &SetTooltip(std::string tooltip);
  ToggleBuilder &SetId(std::string id);

  ToggleBuilder &OnChanged(ToggleChangedAction onChanged);
  ToggleBuilder &Enabled(bool enabled);
  ToggleBuilder &Tooltip(std::string tooltip);
  ToggleBuilder &Id(std::string id);

  operator UIElement() const;

private:
  class Impl;
  std::unique_ptr<Impl> m_Impl;
};

ToggleBuilder Toggle(std::string label, bool value);
} // namespace FlightUI
