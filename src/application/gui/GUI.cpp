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
constexpr const char *GlslVersion = "#version 130";
constexpr int SwapInterval = 1;
constexpr float ClearColorR = 0.10F;
constexpr float ClearColorG = 0.11F;
constexpr float ClearColorB = 0.13F;
constexpr float ClearColorA = 1.00F;
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
  glfwSwapInterval(SwapInterval);

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

  if (!ImGui_ImplOpenGL3_Init(GlslVersion)) {
    std::cerr << "Failed to initialize ImGui OpenGL backend\n";
    Exit();
    return false;
  }
  openGlBackendInitialized_ = true;

  initialized_ = true;
  StartComponents();
  return true;
}

void GUI::Tick() {
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
  TickComponents();
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
  glClearColor(ClearColorR * ClearColorA,
      ClearColorG * ClearColorA,
      ClearColorB * ClearColorA,
      ClearColorA);
  glClear(GL_COLOR_BUFFER_BIT);

  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  glfwSwapBuffers(window_);
}

// private
void GUI::RenderDockSpace() { ImGui::DockSpaceOverViewport(); }

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

  auto &executionControl = GetSimulationExecutionControl();
  const application::SimulationExecutionState executionState =
      executionControl.GetSimulationExecutionState();

  ImGui::BeginDisabled(
      executionState != application::SimulationExecutionState::Running);
  if (ImGui::MenuItem("Pause")) {
    executionControl.PauseSimulation();
  }
  ImGui::EndDisabled();

  ImGui::BeginDisabled(
      executionState != application::SimulationExecutionState::Paused);
  if (ImGui::MenuItem("Resume")) {
    executionControl.ResumeSimulation();
  }
  if (ImGui::MenuItem("Tick Once")) {
    executionControl.RequestSimulationTick();
  }
  ImGui::EndDisabled();

  ImGui::BeginDisabled(
      executionState == application::SimulationExecutionState::Stopped);
  if (ImGui::MenuItem("Reset")) {
    const bool resumeAfterReset =
        executionState == application::SimulationExecutionState::Running;
    executionControl.PauseSimulation();
    if (executionControl.ResetSimulation() && resumeAfterReset) {
      executionControl.ResumeSimulation();
    }
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

void GUI::TickComponents() {
  for (const auto &component : components_) {
    component->Tick(*this);
  }
}
} // namespace gui
