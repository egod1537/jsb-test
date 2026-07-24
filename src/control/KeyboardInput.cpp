#include "control/KeyboardInput.hpp"
#include <algorithm>

#ifdef _WIN32
#include <conio.h>
#else
#include <termios.h>
#include <unistd.h>
#endif

namespace control {
KeyboardInput::~KeyboardInput() {
#ifndef _WIN32
  if (initialized_) {
    tcsetattr(STDIN_FILENO, TCSANOW, &originalTerminal_);
  }
#endif
}

bool KeyboardInput::Initialize() {
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

bool KeyboardInput::Update(ControlInput &input) {
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
      input.elevator -= 0.05;
      changed = true;
      break;
    case 's':
      input.elevator += 0.05;
      changed = true;
      break;
    case 'a':
      input.aileron -= 0.05;
      changed = true;
      break;
    case 'd':
      input.aileron += 0.05;
      changed = true;
      break;
    case 'q':
      input.rudder -= 0.05;
      changed = true;
      break;
    case 'e':
      input.rudder += 0.05;
      changed = true;
      break;
    case 'r':
      input.throttle += 0.05;
      changed = true;
      break;
    case 'f':
      input.throttle -= 0.05;
      changed = true;
      break;
    default:
      break;
    }
  }

  input.elevator = std::clamp(input.elevator, -1.0, 1.0);
  input.aileron = std::clamp(input.aileron, -1.0, 1.0);
  input.rudder = std::clamp(input.rudder, -1.0, 1.0);
  input.throttle = std::clamp(input.throttle, 0.0, 1.0);

  return changed;
}
} // namespace control
