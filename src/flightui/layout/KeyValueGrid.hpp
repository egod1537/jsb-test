#pragma once

#include "flightui/core/UIElement.hpp"

#include <memory>
#include <string>

namespace FlightUI {
class KeyValueGridBuilder {
public:
  explicit KeyValueGridBuilder(std::string id);
  KeyValueGridBuilder(const KeyValueGridBuilder &other);
  KeyValueGridBuilder(KeyValueGridBuilder &&other) noexcept;
  KeyValueGridBuilder &operator=(const KeyValueGridBuilder &other);
  KeyValueGridBuilder &operator=(KeyValueGridBuilder &&other) noexcept;
  ~KeyValueGridBuilder();

  KeyValueGridBuilder &SetColumnsPerRow(int columnsPerRow);
  KeyValueGridBuilder &SetLabelWidth(float width);
  KeyValueGridBuilder &SetEnabled(bool enabled);
  KeyValueGridBuilder &SetVisible(bool visible);
  KeyValueGridBuilder &SetTooltip(std::string tooltip);
  KeyValueGridBuilder &Add(std::string label, std::string value);
  KeyValueGridBuilder &AddDouble(std::string label, double value,
      std::string format);
  KeyValueGridBuilder &AddInt(std::string label, int value, std::string format);

  KeyValueGridBuilder &ColumnsPerRow(int columnsPerRow);
  KeyValueGridBuilder &LabelWidth(float width);
  KeyValueGridBuilder &Enabled(bool enabled);
  KeyValueGridBuilder &Visible(bool visible);
  KeyValueGridBuilder &Tooltip(std::string tooltip);

  operator UIElement() const;

private:
  class Impl;
  std::unique_ptr<Impl> m_Impl;
};

KeyValueGridBuilder KeyValueGrid(std::string id);
} // namespace FlightUI
