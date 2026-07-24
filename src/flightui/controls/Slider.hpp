#pragma once

#include "flightui/core/UIElement.hpp"

#include <imgui.h>

#include <functional>
#include <memory>
#include <string>

namespace FlightUI {
using SliderFloatChangedAction = std::function<void(float)>;
using SliderDoubleChangedAction = std::function<void(double)>;
using SliderIntChangedAction = std::function<void(int)>;

class SliderFloatBuilder {
public:
  SliderFloatBuilder(std::string label, float value, float minimum,
                     float maximum);
  SliderFloatBuilder(const SliderFloatBuilder &other);
  SliderFloatBuilder(SliderFloatBuilder &&other) noexcept;
  SliderFloatBuilder &operator=(const SliderFloatBuilder &other);
  SliderFloatBuilder &operator=(SliderFloatBuilder &&other) noexcept;
  ~SliderFloatBuilder();

  SliderFloatBuilder &SetOnChanged(SliderFloatChangedAction onChanged);
  SliderFloatBuilder &SetFormat(std::string format);
  SliderFloatBuilder &SetWidth(float width);
  SliderFloatBuilder &SetEnabled(bool enabled);
  SliderFloatBuilder &SetFlags(ImGuiSliderFlags flags);
  SliderFloatBuilder &SetTooltip(std::string tooltip);
  SliderFloatBuilder &SetId(std::string id);

  SliderFloatBuilder &OnChanged(SliderFloatChangedAction onChanged);
  SliderFloatBuilder &Format(std::string format);
  SliderFloatBuilder &Width(float width);
  SliderFloatBuilder &Enabled(bool enabled);
  SliderFloatBuilder &Flags(ImGuiSliderFlags flags);
  SliderFloatBuilder &Tooltip(std::string tooltip);
  SliderFloatBuilder &Id(std::string id);

  operator UIElement() const;

private:
  class Impl;
  std::unique_ptr<Impl> m_Impl;
};

class SliderDoubleBuilder {
public:
  SliderDoubleBuilder(std::string label, double value, double minimum,
                      double maximum);
  SliderDoubleBuilder(const SliderDoubleBuilder &other);
  SliderDoubleBuilder(SliderDoubleBuilder &&other) noexcept;
  SliderDoubleBuilder &operator=(const SliderDoubleBuilder &other);
  SliderDoubleBuilder &operator=(SliderDoubleBuilder &&other) noexcept;
  ~SliderDoubleBuilder();

  SliderDoubleBuilder &SetOnChanged(SliderDoubleChangedAction onChanged);
  SliderDoubleBuilder &SetFormat(std::string format);
  SliderDoubleBuilder &SetWidth(float width);
  SliderDoubleBuilder &SetEnabled(bool enabled);
  SliderDoubleBuilder &SetFlags(ImGuiSliderFlags flags);
  SliderDoubleBuilder &SetTooltip(std::string tooltip);
  SliderDoubleBuilder &SetId(std::string id);

  SliderDoubleBuilder &OnChanged(SliderDoubleChangedAction onChanged);
  SliderDoubleBuilder &Format(std::string format);
  SliderDoubleBuilder &Width(float width);
  SliderDoubleBuilder &Enabled(bool enabled);
  SliderDoubleBuilder &Flags(ImGuiSliderFlags flags);
  SliderDoubleBuilder &Tooltip(std::string tooltip);
  SliderDoubleBuilder &Id(std::string id);

  operator UIElement() const;

private:
  class Impl;
  std::unique_ptr<Impl> m_Impl;
};

class SliderIntBuilder {
public:
  SliderIntBuilder(std::string label, int value, int minimum, int maximum);
  SliderIntBuilder(const SliderIntBuilder &other);
  SliderIntBuilder(SliderIntBuilder &&other) noexcept;
  SliderIntBuilder &operator=(const SliderIntBuilder &other);
  SliderIntBuilder &operator=(SliderIntBuilder &&other) noexcept;
  ~SliderIntBuilder();

  SliderIntBuilder &SetOnChanged(SliderIntChangedAction onChanged);
  SliderIntBuilder &SetFormat(std::string format);
  SliderIntBuilder &SetWidth(float width);
  SliderIntBuilder &SetEnabled(bool enabled);
  SliderIntBuilder &SetFlags(ImGuiSliderFlags flags);
  SliderIntBuilder &SetTooltip(std::string tooltip);
  SliderIntBuilder &SetId(std::string id);

  SliderIntBuilder &OnChanged(SliderIntChangedAction onChanged);
  SliderIntBuilder &Format(std::string format);
  SliderIntBuilder &Width(float width);
  SliderIntBuilder &Enabled(bool enabled);
  SliderIntBuilder &Flags(ImGuiSliderFlags flags);
  SliderIntBuilder &Tooltip(std::string tooltip);
  SliderIntBuilder &Id(std::string id);

  operator UIElement() const;

private:
  class Impl;
  std::unique_ptr<Impl> m_Impl;
};

SliderFloatBuilder SliderFloat(std::string label, float value, float minimum,
                               float maximum);
SliderDoubleBuilder SliderDouble(std::string label, double value,
                                 double minimum, double maximum);
SliderIntBuilder SliderInt(std::string label, int value, int minimum,
                           int maximum);
} // namespace FlightUI
