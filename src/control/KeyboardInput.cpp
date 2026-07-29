#include "control/KeyboardInput.hpp"

#include "control/ControlInputStrategy.hpp"

#ifdef _WIN32
#include <conio.h>
#else
#include <termios.h>
#include <unistd.h>
#endif

namespace control {
namespace {
constexpr double InputStep = 0.05;
} // namespace

KeyboardInput::~KeyboardInput() { Shutdown(); }

void KeyboardInput::Shutdown() {
#ifndef _WIN32
  if (initialized_) {
    tcsetattr(STDIN_FILENO, TCSANOW, &originalTerminal_);
  }
#endif

  initialized_ = false;
}

bool KeyboardInput::Initialize() {
  if (initialized_) {
    return true;
  }

#ifdef _WIN32
  initialized_ = true;
  return true;
#else
  if (tcgetattr(STDIN_FILENO, &originalTerminal_) != 0) {
    return false;
  }

  termios rawTerminal = originalTerminal_;

  rawTerminal.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
  rawTerminal.c_cc[VMIN] = 0;
  rawTerminal.c_cc[VTIME] = 0;

  if (tcsetattr(STDIN_FILENO, TCSANOW, &rawTerminal) != 0) {
    return false;
  }

  initialized_ = true;
  return true;
#endif
}

bool KeyboardInput::Update(ControlInputStrategy &strategy) {
  char key = '\0';
  bool changed = false;

#ifdef _WIN32
  while (_kbhit() != 0) {
    key = static_cast<char>(_getch());
#else
  while (read(STDIN_FILENO, &key, 1) > 0) {
#endif
    switch (key) {
    case 'w':
      changed = strategy.AdjustCommandedInput(ControlAxis::Elevator, -InputStep)
                || changed;
      break;
    case 's':
      changed = strategy.AdjustCommandedInput(ControlAxis::Elevator, InputStep)
                || changed;
      break;
    case 'a':
      changed = strategy.AdjustCommandedInput(ControlAxis::Aileron, -InputStep)
                || changed;
      break;
    case 'd':
      changed = strategy.AdjustCommandedInput(ControlAxis::Aileron, InputStep)
                || changed;
      break;
    case 'q':
      changed = strategy.AdjustCommandedInput(ControlAxis::Rudder, -InputStep)
                || changed;
      break;
    case 'e':
      changed = strategy.AdjustCommandedInput(ControlAxis::Rudder, InputStep)
                || changed;
      break;
    case 'r':
      changed = strategy.AdjustCommandedInput(ControlAxis::Throttle, InputStep)
                || changed;
      break;
    case 'f':
      changed = strategy.AdjustCommandedInput(ControlAxis::Throttle, -InputStep)
                || changed;
      break;
    default:
      break;
    }
  }

  return changed;
}
} // namespace control
