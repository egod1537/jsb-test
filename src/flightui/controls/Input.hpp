#pragma once

#include "flightui/core/UIElement.hpp"

#include <functional>
#include <memory>
#include <string>

namespace FlightUI {
using InputDoubleChangedAction = std::function<void(double)>;

class InputDoubleBuilder {
public:
  InputDoubleBuilder(std::string label, double value);
  InputDoubleBuilder(const InputDoubleBuilder &other);
  InputDoubleBuilder(InputDoubleBuilder &&other) noexcept;
  InputDoubleBuilder &operator=(const InputDoubleBuilder &other);
  InputDoubleBuilder &operator=(InputDoubleBuilder &&other) noexcept;
  ~InputDoubleBuilder();

  InputDoubleBuilder &SetOnChanged(InputDoubleChangedAction onChanged);
  InputDoubleBuilder &SetStep(double step);
  InputDoubleBuilder &SetFastStep(double step);
  InputDoubleBuilder &SetFormat(std::string format);
  InputDoubleBuilder &SetWidth(float width);
  InputDoubleBuilder &SetEnabled(bool enabled);
  InputDoubleBuilder &SetTooltip(std::string tooltip);
  InputDoubleBuilder &SetId(std::string id);

  InputDoubleBuilder &OnChanged(InputDoubleChangedAction onChanged);
  InputDoubleBuilder &Step(double step);
  InputDoubleBuilder &FastStep(double step);
  InputDoubleBuilder &Format(std::string format);
  InputDoubleBuilder &Width(float width);
  InputDoubleBuilder &Enabled(bool enabled);
  InputDoubleBuilder &Tooltip(std::string tooltip);
  InputDoubleBuilder &Id(std::string id);

  operator UIElement() const;

private:
  class Impl;
  std::unique_ptr<Impl> m_Impl;
};

InputDoubleBuilder InputDouble(std::string label, double value);
} // namespace FlightUI
