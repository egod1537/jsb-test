#pragma once

#include <string>

namespace gui {
struct GUIConfig {
  int windowWidth = 1280;
  int windowHeight = 720;
  std::string windowTitle = "JSB Flight Console";

  double renderHz = 60.0;
  bool vsync = true;

  float clearColorR = 0.10F;
  float clearColorG = 0.11F;
  float clearColorB = 0.13F;
  float clearColorA = 1.00F;

  double GetRenderDT() const { return renderHz > 0.0 ? 1.0 / renderHz : 0.0; }
};
} // namespace gui
