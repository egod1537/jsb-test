cmake_minimum_required(VERSION 3.28)

project(
  jsb-flight-console
  LANGUAGES CXX
)

include(FetchContent)

FetchContent_Declare(
  jsbsim
  GIT_REPOSITORY https://github.com/JSBSim-Team/jsbsim.git
  GIT_TAG v1.3.0
)
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG docking
)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 5.0.1
)
FetchContent_Declare(
  implot
  GIT_REPOSITORY https://github.com/epezent/implot.git
  GIT_TAG v1.0
)

FetchContent_MakeAvailable(jsbsim imgui eigen implot)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# imgui
add_library(imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui PUBLIC
  glfw
  OpenGL::GL
)

#implot
add_library(implot STATIC
  ${implot_SOURCE_DIR}/implot.cpp
  ${implot_SOURCE_DIR}/implot_items.cpp
)

target_include_directories(implot PUBLIC
  ${implot_SOURCE_DIR}
)

target_link_libraries(implot PUBLIC
  imgui
)

add_library(
  flightui STATIC
  src/flightui/core/UICommon.cpp
  src/flightui/core/UIElement.cpp
  src/flightui/layout/Window.cpp
  src/flightui/layout/Panel.cpp
  src/flightui/layout/FoldOut.cpp
  src/flightui/layout/KeyValueGrid.cpp
  src/flightui/layout/TabGroup.cpp
  src/flightui/layout/VerticalLayout.cpp
  src/flightui/layout/HorizontalLayout.cpp
  src/flightui/controls/Text.cpp
  src/flightui/controls/Heading.cpp
  src/flightui/controls/ValueLabel.cpp
  src/flightui/controls/Button.cpp
  src/flightui/controls/Combo.cpp
  src/flightui/controls/Toggle.cpp
  src/flightui/controls/Input.cpp
  src/flightui/controls/Slider.cpp
  src/flightui/controls/Separator.cpp
  src/flightui/controls/Space.cpp
  src/flightui/controls/Custom.cpp
  src/flightui/plot/DataView.cpp
  src/flightui/plot/Plot.cpp
)

target_compile_features(
  flightui
  PUBLIC
  cxx_std_20
)

target_link_libraries(
  flightui
  PUBLIC
  imgui
  implot
)

target_include_directories(
  flightui
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(
  jsb-flight-console
  src/main.cpp
  src/application/Application.cpp
  src/application/input/Input.cpp
  src/application/sim/Aircraft.cpp
  src/application/sim/Component.cpp
  src/application/sim/ErrorTracker.cpp
  src/application/sim/InitialCondition.cpp
  src/application/sim/StateLogger.cpp
  src/application/sim/control/ManualFlightControlController.cpp
  src/application/sim/control/FlightControlManager.cpp
  src/application/sim/gnc/TrimSolver.cpp
  src/application/sim/gnc/Autopilot.cpp
  src/application/sim/gnc/hold/RollHoldController.cpp
  src/application/sim/gnc/hold/PitchHoldController.cpp
  src/application/sim/gnc/hold/AirspeedHoldController.cpp
  src/application/sim/gnc/hold/CourseHoldController.cpp
  src/application/sim/gnc/hold/AltitudeHoldController.cpp
  src/application/gui/viz/core/Component.cpp
  src/application/gui/viz/core/GameObject.cpp
  src/application/gui/viz/core/Scene.cpp
  src/application/gui/viz/render/CameraComponent.cpp
  src/application/gui/viz/render/LineCanvas.cpp
  src/application/gui/viz/components/GroundGridRenderer.cpp
  src/application/gui/viz/components/AltitudeCueRenderer.cpp
  src/application/gui/viz/components/FlightCameraController.cpp
  src/application/gui/viz/components/AircraftWireframeRenderer.cpp
  src/application/gui/viz/components/TelemetryOverlay.cpp
  src/application/gui/viz/FlightVisualizer.cpp
  src/application/gui/GUI.cpp
  src/application/gui/Window.cpp
  src/application/gui/panels/AutopilotPanel.cpp
  src/application/gui/panels/ManualControlPanel.cpp
  src/application/gui/panels/TrimPanel.cpp
  src/application/gui/windows/SimulationWindow.cpp
  src/application/gui/windows/FlightConsoleWindow.cpp
  src/application/gui/windows/monitor/FlightDataMonitorWindow.cpp
  src/application/gui/windows/viz/FlightVizWindow.cpp
  src/application/gui/windows/GNCWindow.cpp
  src/application/gui/windows/samples/DeclarativeDemoWindow.cpp
  src/application/gui/windows/samples/SamplePlotWindow.cpp
  src/application/sim/jsbsim/ControlSystem.cpp
  src/application/sim/jsbsim/Properties.cpp
  src/application/sim/jsbsim/EngineSystem.cpp
  src/application/flightgear/FlightGearSender.cpp
  src/application/flightgear/FlightGearSystem.cpp
  src/application/sim/Simulation.cpp
)

target_compile_features(
  jsb-flight-console
  PRIVATE
  cxx_std_20
)

target_link_libraries(
  jsb-flight-console
  PRIVATE
  libJSBSim
  Eigen3::Eigen
  flightui
)

if(WIN32)
  target_link_libraries(
    jsb-flight-console
    PRIVATE
    ws2_32
  )
endif()

enable_testing()

add_executable(
  flightui_tests
  tests/FlightUITests.cpp
)

target_compile_features(
  flightui_tests
  PRIVATE
  cxx_std_20
)

target_link_libraries(
  flightui_tests
  PRIVATE
  flightui
)

add_test(
  NAME flightui_tests
  COMMAND flightui_tests
)

add_executable(
  simulation_tests
  tests/SimulationTests.cpp
  src/application/input/Input.cpp
  src/application/sim/Simulation.cpp
  src/application/sim/Aircraft.cpp
  src/application/sim/Component.cpp
  src/application/sim/ErrorTracker.cpp
  src/application/sim/InitialCondition.cpp
  src/application/sim/StateLogger.cpp
  src/application/sim/jsbsim/ControlSystem.cpp
  src/application/sim/jsbsim/Properties.cpp
  src/application/sim/jsbsim/EngineSystem.cpp
  src/application/sim/control/ManualFlightControlController.cpp
  src/application/sim/control/FlightControlManager.cpp
  src/application/sim/gnc/TrimSolver.cpp
  src/application/sim/gnc/Autopilot.cpp
  src/application/sim/gnc/hold/RollHoldController.cpp
  src/application/sim/gnc/hold/PitchHoldController.cpp
  src/application/sim/gnc/hold/AirspeedHoldController.cpp
  src/application/sim/gnc/hold/CourseHoldController.cpp
  src/application/sim/gnc/hold/AltitudeHoldController.cpp
)

target_compile_features(
  simulation_tests
  PRIVATE
  cxx_std_20
)

target_include_directories(
  simulation_tests
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(
  simulation_tests
  PRIVATE
  libJSBSim
  Eigen3::Eigen
)

add_test(
  NAME simulation_tests
  COMMAND simulation_tests
)

target_include_directories(
  jsb-flight-console
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
