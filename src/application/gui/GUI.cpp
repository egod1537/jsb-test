#include "GUI.hpp"
#include "application/gui/viz/FlightVisualizer.hpp"
#include "application/gui/windows/GNCWindow.hpp"
#include "application/gui/windows/SimulationWindow.hpp"
#include "application/gui/windows/monitor/FlightDataMonitorWindow.hpp"
#include "application/gui/windows/viz/FlightVizWindow.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include <iostream>
#include <utility>

namespace {
constexpr const char *GLSL_VERSION = "#version 130";
} // namespace

namespace gui {
// public
GUI::GUI(sim::Simulation *sim, GUIConfig config)
    : sim_(sim), visualizer_(std::make_unique<viz::FlightVisualizer>()),
      config_(std::move(config)) {
  RegisterWindow<SimulationWindow>();
  RegisterWindow<GNCWindow>();
  RegisterWindow<FlightDataMonitorWindow>();
  RegisterWindow<FlightVizWindow>();
}

GUI::~GUI() { Exit(); }

bool GUI::Start() {
  if (initialized_) {
    return true;
  }

  if (glfwInit() == GLFW_FALSE) {
    std::cerr << "Failed to initialize GLFW\n";
    return false;
  }
  glfwInitialized_ = true;

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  window_ = glfwCreateWindow(config_.windowWidth,
      config_.windowHeight,
      config_.windowTitle.c_str(),
      nullptr,
      nullptr);

  if (window_ == nullptr) {
    std::cerr << "Failed to create GLFW window\n";
    Exit();
    return false;
  }

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(config_.vsync ? 1 : 0);

  IMGUI_CHECKVERSION();

  ImGui::CreateContext();
  ImPlot::CreateContext();
  imguiContextCreated_ = true;

  ImGuiIO &io = ImGui::GetIO();

  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

  ImGui::StyleColorsDark();

  if (!ImGui_ImplGlfw_InitForOpenGL(window_, true)) {
    std::cerr << "Failed to initialize ImGui GLFW backend\n";
    Exit();
    return false;
  }
  glfwBackendInitialized_ = true;

  if (!ImGui_ImplOpenGL3_Init(GLSL_VERSION)) {
    std::cerr << "Failed to initialize ImGui OpenGL backend\n";
    Exit();
    return false;
  }
  openGlBackendInitialized_ = true;

  initialized_ = true;
  StartComponents();
  return true;
}

void GUI::Update() {
  if (!initialized_) {
    return;
  }

  BeginFrame();
  RenderFrame();
  EndFrame();
}

void GUI::Exit() {
  if (openGlBackendInitialized_) {
    ImGui_ImplOpenGL3_Shutdown();
    openGlBackendInitialized_ = false;
  }

  if (glfwBackendInitialized_) {
    ImGui_ImplGlfw_Shutdown();
    glfwBackendInitialized_ = false;
  }

  if (imguiContextCreated_) {
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    imguiContextCreated_ = false;
  }

  initialized_ = false;

  if (window_ != nullptr) {
    glfwDestroyWindow(window_);
    window_ = nullptr;
  }

  if (glfwInitialized_) {
    glfwTerminate();
    glfwInitialized_ = false;
  }
}

void GUI::RegisterComponent(std::unique_ptr<Component> component) {
  if (component == nullptr) {
    return;
  }

  components_.push_back(std::move(component));
  if (initialized_) {
    components_.back()->StartIfNeeded(*this);
  }
}

void GUI::RegisterWindow(std::unique_ptr<Window> window) {
  if (window == nullptr) {
    return;
  }

  windows_.push_back(window.get());
  RegisterComponent(std::move(window));
}

bool GUI::ShouldClose() const {
  return window_ == nullptr || glfwWindowShouldClose(window_);
}

void GUI::RequestClose() {
  if (window_ != nullptr) {
    glfwSetWindowShouldClose(window_, GLFW_TRUE);
  }
}

void GUI::BeginFrame() {
  if (!initialized_) {
    return;
  }

  glfwPollEvents();
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void GUI::RenderFrame() {
  if (!initialized_) {
    return;
  }

  RenderMainMenuBar();
  RenderDockSpace();
  UpdateComponents();
}

void GUI::EndFrame() {
  if (!initialized_) {
    return;
  }

  ImGui::Render();

  int displayWidth = 0;
  int displayHeight = 0;
  glfwGetFramebufferSize(window_, &displayWidth, &displayHeight);

  glViewport(0, 0, displayWidth, displayHeight);
  glClearColor(config_.clearColorR * config_.clearColorA,
      config_.clearColorG * config_.clearColorA,
      config_.clearColorB * config_.clearColorA,
      config_.clearColorA);
  glClear(GL_COLOR_BUFFER_BIT);

  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  glfwSwapBuffers(window_);
}

// private
void GUI::RenderDockSpace() {
  ImGui::DockSpaceOverViewport();
}

void GUI::RenderMainMenuBar() {
  if (!ImGui::BeginMainMenuBar()) {
    return;
  }

  RenderSimulationMenu();
  RenderWindowMenu();

  ImGui::EndMainMenuBar();
}

void GUI::RenderSimulationMenu() {
  if (!ImGui::BeginMenu("Simulation")) {
    return;
  }

  auto &simulation = GetSimulation();

  ImGui::BeginDisabled(!simulation.IsRunning());
  if (ImGui::MenuItem("Pause")) {
    simulation.Pause();
  }
  ImGui::EndDisabled();

  ImGui::BeginDisabled(!simulation.IsPaused());
  if (ImGui::MenuItem("Resume")) {
    simulation.Resume();
  }
  if (ImGui::MenuItem("Step Once")) {
    simulation.RequestStep();
  }
  ImGui::EndDisabled();

  ImGui::BeginDisabled(simulation.IsStopped());
  if (ImGui::MenuItem("Restart")) {
    simulation.Restart();
  }
  ImGui::EndDisabled();

  ImGui::Separator();

  if (ImGui::MenuItem("Exit")) {
    RequestClose();
  }

  ImGui::EndMenu();
}

void GUI::RenderWindowMenu() {
  if (!ImGui::BeginMenu("Window")) {
    return;
  }

  for (Window *window : windows_) {
    ImGui::MenuItem(window->GetTitle().c_str(),
        nullptr,
        window->GetVisiblePtr());
  }

  ImGui::Separator();

  if (ImGui::MenuItem("Show All")) {
    for (Window *window : windows_) {
      window->SetVisible(true);
    }
  }

  if (ImGui::MenuItem("Hide All")) {
    for (Window *window : windows_) {
      window->SetVisible(false);
    }
  }

  ImGui::EndMenu();
}

void GUI::StartComponents() {
  for (const auto &component : components_) {
    component->StartIfNeeded(*this);
  }
}

void GUI::UpdateComponents() {
  for (const auto &component : components_) {
    component->Tick(*this);
  }
}
} // namespace gui
