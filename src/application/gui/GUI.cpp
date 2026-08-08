#include "GUI.hpp"
#include "application/gui/viz/FlightVisualizer.hpp"
#include "application/gui/windows/GNCWindow.hpp"
#include "application/gui/windows/LinearizationWindow.hpp"
#include "application/gui/windows/SimulationWindow.hpp"
#include "application/gui/windows/monitor/FlightDataMonitorWindow.hpp"
#include "application/gui/windows/viz/FlightVizWindow.hpp"
#include "flightui/core/Theme.hpp"
#include "flightui/core/UIFont.hpp"
#include "flightui/core/UIScale.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include <cmath>
#include <iostream>
#include <utility>

namespace {
constexpr const char *GlslVersion = "#version 130";
constexpr int SwapInterval = 1;
constexpr float UIScaleChangeThreshold = 0.02F;

ImVec2 Scaled(ImVec2 value, float scale) {
  return {value.x * scale, value.y * scale};
}

void ScaleImPlotStyle(ImPlotStyle &style, float scale) {
  style.PlotDefaultSize = Scaled(style.PlotDefaultSize, scale);
  style.PlotMinSize = Scaled(style.PlotMinSize, scale);
  style.PlotBorderSize *= scale;
  style.MajorTickLen = Scaled(style.MajorTickLen, scale);
  style.MinorTickLen = Scaled(style.MinorTickLen, scale);
  style.MajorTickSize = Scaled(style.MajorTickSize, scale);
  style.MinorTickSize = Scaled(style.MinorTickSize, scale);
  style.MajorGridSize = Scaled(style.MajorGridSize, scale);
  style.MinorGridSize = Scaled(style.MinorGridSize, scale);
  style.PlotPadding = Scaled(style.PlotPadding, scale);
  style.LabelPadding = Scaled(style.LabelPadding, scale);
  style.LegendPadding = Scaled(style.LegendPadding, scale);
  style.LegendInnerPadding = Scaled(style.LegendInnerPadding, scale);
  style.LegendSpacing = Scaled(style.LegendSpacing, scale);
  style.MousePosPadding = Scaled(style.MousePosPadding, scale);
  style.AnnotationPadding = Scaled(style.AnnotationPadding, scale);
  style.DigitalPadding *= scale;
  style.DigitalSpacing *= scale;
}
} // namespace

namespace gui {
// public
GUI::GUI(sim::Simulation *sim, GUIConfig config)
    : sim_(sim), visualizer_(std::make_unique<viz::FlightVisualizer>()),
      config_(std::move(config)) {
  RegisterWindow<SimulationWindow>();
  RegisterWindow<GNCWindow>();
  RegisterWindow<LinearizationWindow>();
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
  // DPI changes affect raster density through the GLFW/OpenGL backends. Keep
  // logical font sizing tied only to the responsive window-resolution scale.
  io.ConfigDpiScaleFonts = false;

  FlightUI::ApplyDarkEditorTheme();
  FlightUI::LoadPrimaryUIFont();
  baseImGuiStyle_ = ImGui::GetStyle();
  baseImPlotStyle_ = ImPlot::GetStyle();
  UpdateUIScale(true);

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
  UpdateUIScale();
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

  const ImVec4 clearColor = FlightUI::GetDarkEditorApplicationBackground();
  glViewport(0, 0, displayWidth, displayHeight);
  glClearColor(clearColor.x * clearColor.w,
      clearColor.y * clearColor.w,
      clearColor.z * clearColor.w,
      clearColor.w);
  glClear(GL_COLOR_BUFFER_BIT);

  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  glfwSwapBuffers(window_);
}

// private
void GUI::UpdateUIScale(bool force) {
  int windowWidth = 0;
  int windowHeight = 0;
  glfwGetWindowSize(window_, &windowWidth, &windowHeight);
  if (windowWidth <= 0 || windowHeight <= 0) {
    return;
  }

  const float uiScale =
      FlightUI::CalculateUIScale(static_cast<float>(windowWidth),
          static_cast<float>(windowHeight));
  if (!force && std::abs(uiScale - appliedUIScale_) < UIScaleChangeThreshold) {
    return;
  }

  appliedUIScale_ = uiScale;
  FlightUI::SetUIScale(uiScale);

  ImGuiStyle scaledImGuiStyle = baseImGuiStyle_;
  scaledImGuiStyle.ScaleAllSizes(uiScale);
  // The current ImGui backend rasterizes dynamically requested font sizes,
  // while framebuffer density remains a separate backend concern.
  scaledImGuiStyle.FontScaleMain =
      baseImGuiStyle_.FontScaleMain * FlightUI::CalculateUIFontScale(uiScale);
  ImGui::GetStyle() = scaledImGuiStyle;

  ImPlotStyle scaledImPlotStyle = baseImPlotStyle_;
  ScaleImPlotStyle(scaledImPlotStyle, uiScale);
  ImPlot::GetStyle() = scaledImPlotStyle;
}

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
