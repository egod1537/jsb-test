#pragma once

#include "flightui/core/UIElement.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace FlightUI {
using ComboChangedAction = std::function<void(int)>;

class ComboBuilder {
public:
  ComboBuilder(std::string label, int selectedIndex,
      std::vector<std::string> items);
  ComboBuilder(const ComboBuilder &other);
  ComboBuilder(ComboBuilder &&other) noexcept;
  ComboBuilder &operator=(const ComboBuilder &other);
  ComboBuilder &operator=(ComboBuilder &&other) noexcept;
  ~ComboBuilder();

  ComboBuilder &SetOnChanged(ComboChangedAction onChanged);
  ComboBuilder &SetWidth(float width);
  ComboBuilder &SetEnabled(bool enabled);
  ComboBuilder &SetTooltip(std::string tooltip);
  ComboBuilder &SetId(std::string id);

  ComboBuilder &OnChanged(ComboChangedAction onChanged);
  ComboBuilder &Width(float width);
  ComboBuilder &Enabled(bool enabled);
  ComboBuilder &Tooltip(std::string tooltip);
  ComboBuilder &Id(std::string id);

  operator UIElement() const;

private:
  class Impl;
  std::unique_ptr<Impl> m_Impl;
};

ComboBuilder Combo(std::string label, int selectedIndex,
    std::vector<std::string> items);
} // namespace FlightUI
