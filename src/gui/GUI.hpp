#pragma once

#include "gui/Component.hpp"
#include "gui/GUIConfig.hpp"
#include "gui/Window.hpp"
#include "simulation/Simulation.hpp"
#include <GLFW/glfw3.h>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

namespace gui {
class GUI {
public:
  explicit GUI(sim::Simulation *sim, GUIConfig config = {});
  ~GUI();

  GUI(const GUI &) = delete;
  GUI &operator=(const GUI &) = delete;

  bool Start();
  void Update();
  void Exit();

  bool ShouldClose() const;

  const GUIConfig &GetConfig() const { return config_; }

  sim::Simulation &GetSimulation() { return *sim_; }
  const sim::Simulation &GetSimulation() const { return *sim_; }

  void RegisterComponent(std::unique_ptr<Component> component);
  void RegisterWindow(std::unique_ptr<Window> window);

  template <typename T, typename... Args> T &RegisterComponent(Args &&...args) {
    static_assert(std::is_base_of_v<Component, T>,
        "T must inherit from gui::Component");

    auto component = std::make_unique<T>(std::forward<Args>(args)...);
    T &componentRef = *component;
    RegisterComponent(std::move(component));
    return componentRef;
  }

  template <typename T, typename... Args> T &RegisterWindow(Args &&...args) {
    static_assert(std::is_base_of_v<Window, T>,
        "T must inherit from gui::Window");

    auto window = std::make_unique<T>(std::forward<Args>(args)...);
    T &windowRef = *window;
    RegisterWindow(std::move(window));
    return windowRef;
  }

private:
  void BeginFrame();
  void RenderFrame();
  void EndFrame();

  void StartComponents();
  void UpdateComponents();

  GLFWwindow *window_ = nullptr;
  bool initialized_ = false;
  bool glfwInitialized_ = false;
  bool imguiContextCreated_ = false;
  bool glfwBackendInitialized_ = false;
  bool openGlBackendInitialized_ = false;

  std::vector<std::unique_ptr<Component>> components_;
  sim::Simulation *sim_;
  GUIConfig config_;
};
} // namespace gui
