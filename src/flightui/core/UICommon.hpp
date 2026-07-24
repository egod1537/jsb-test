#pragma once

#include <functional>

namespace FlightUI {
struct Vector2 {
  float X = 0.0F;
  float Y = 0.0F;
};

using Action = std::function<void()>;
} // namespace FlightUI
