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
  src/simulation/Aircraft.cpp
  src/simulation/Context.cpp
  src/gnc/TrimSolver.cpp
  src/gnc/RollHoldSystem.cpp
  src/state/TrueStateProvider.cpp
  src/gui/GUI.cpp
  src/gui/Window.cpp
  src/gui/windows/SimulationWindow.cpp
  src/gui/windows/FlightConsoleWindow.cpp
  src/gui/windows/monitor/FlightDataMonitorWindow.cpp
  src/gui/windows/GNCWindow.cpp
  src/gui/windows/samples/DeclarativeDemoWindow.cpp
  src/gui/windows/samples/SamplePlotWindow.cpp
  src/jsbsim/FlightControls.cpp
  src/jsbsim/FlightProperties.cpp
  src/flightgear/FlightGearSender.cpp
  src/flightgear/FlightGearSystem.cpp
  src/simulation/Simulation.cpp
  src/control/KeyboardInput.cpp
  src/control/KeyboardInputSystem.cpp
  src/control/ManualControlInputStrategy.cpp
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
  src/simulation/Simulation.cpp
  src/simulation/Aircraft.cpp
  src/simulation/Context.cpp
  src/state/TrueStateProvider.cpp
  src/jsbsim/FlightControls.cpp
  src/jsbsim/FlightProperties.cpp
  src/flightgear/FlightGearSender.cpp
  src/flightgear/FlightGearSystem.cpp
  src/control/KeyboardInput.cpp
  src/control/KeyboardInputSystem.cpp
  src/control/ManualControlInputStrategy.cpp
  src/gnc/RollHoldSystem.cpp
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

if(WIN32)
  target_link_libraries(
    simulation_tests
    PRIVATE
    ws2_32
  )
endif()

add_test(
  NAME simulation_tests
  COMMAND simulation_tests
)

target_include_directories(
  jsb-flight-console
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
