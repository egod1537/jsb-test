#pragma once

#ifndef _WIN32
#include <termios.h>
#endif

namespace control {
class ControlInputStrategy;

class KeyboardInput {
public:
  KeyboardInput() = default;
  ~KeyboardInput();

  KeyboardInput(const KeyboardInput &) = delete;
  KeyboardInput &operator=(const KeyboardInput &) = delete;

  bool Initialize();
  void Shutdown();
  bool Update(ControlInputStrategy &strategy);

private:
#ifndef _WIN32
  termios originalTerminal_{};
#endif
  bool initialized_ = false;
};
} // namespace control
