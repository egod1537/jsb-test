#pragma once

#include "control/ControlInput.hpp"

#ifndef _WIN32
#include <termios.h>
#endif

namespace control {
class KeyboardInput {
public:
  KeyboardInput() = default;
  ~KeyboardInput();

  KeyboardInput(const KeyboardInput &) = delete;
  KeyboardInput &operator=(const KeyboardInput &) = delete;

  bool Initialize();
  bool Update(ControlInput &input);

private:
#ifndef _WIN32
  termios originalTerminal_{};
#endif
  bool initialized_ = false;
};
} // namespace control
