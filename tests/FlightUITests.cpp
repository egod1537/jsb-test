#include "flightui/FlightUI.hpp"

#include <cassert>
#include <imgui.h>
#include <implot.h>
#include <string>
#include <utility>
#include <vector>

namespace UI = FlightUI;

template <typename T>
concept CanSliderDouble =
    requires { UI::SliderDouble("Value", std::declval<T>(), 0.0, 1.0); };

template <typename T>
concept CanSliderFloat =
    requires { UI::SliderFloat("Value", std::declval<T>(), 0.0F, 1.0F); };

template <typename T>
concept CanSliderInt =
    requires { UI::SliderInt("Value", std::declval<T>(), 0, 10); };

template <typename T>
concept CanToggle = requires { UI::Toggle("Enabled", std::declval<T>()); };

template <typename T>
concept CanMakeDataView = requires { UI::DataView::From(std::declval<T>()); };

template <typename T>
concept CanMakeRingBufferDataView =
    requires(const T &buffer) { buffer.data_view(); };

static_assert(CanSliderDouble<double>);
static_assert(CanSliderDouble<const double &>);
static_assert(CanSliderDouble<double &&>);
static_assert(CanSliderFloat<float>);
static_assert(CanSliderFloat<const float &>);
static_assert(CanSliderFloat<float &&>);
static_assert(CanSliderInt<int>);
static_assert(CanSliderInt<const int &>);
static_assert(CanSliderInt<int &&>);
static_assert(CanToggle<bool>);
static_assert(CanToggle<const bool &>);
static_assert(CanToggle<bool &&>);
static_assert(CanMakeDataView<const std::vector<double> &>);
static_assert(!CanMakeDataView<std::vector<double> &&>);
static_assert(CanMakeDataView<const std::vector<float> &>);
static_assert(!CanMakeDataView<std::vector<float> &&>);
static_assert(CanMakeRingBufferDataView<ds::RingBuffer<double>>);
static_assert(CanMakeRingBufferDataView<ds::RingBuffer<float>>);
static_assert(!CanMakeRingBufferDataView<ds::RingBuffer<int>>);
static_assert(requires {
  UI::Button("Reset")
      .OnAction([] {})
      .Width(80.0F)
      .Height(24.0F)
      .Enabled(true)
      .Tooltip("Reset controls")
      .Id("reset-button");
  UI::Button("Typo Alias").Widht(80.0F);
});
static_assert(requires {
  UI::Toggle("Enabled", true).OnChanged([](bool) {});
  UI::SliderFloat("Value", 0.5F, 0.0F, 1.0F).OnChanged([](float) {});
  UI::SliderDouble("Value", 0.5, 0.0, 1.0).OnChanged([](double) {});
  UI::SliderInt("Value", 5, 0, 10).OnChanged([](int) {});
});
static_assert(requires(bool isOpen) {
  UI::FoldOut("Advanced")
      .Open(isOpen)
      .DefaultOpen()
      .Flags(ImGuiTreeNodeFlags_Framed)
      .Enabled(true)
      .Visible(true)
      .Tooltip("Advanced settings")
      .Id("advanced")[UI::Text("Fold out content")];
  UI::TabGroup("Main Tabs")
      .Flags(ImGuiTabBarFlags_Reorderable)
      .Enabled(true)
      .Visible(true)
      .Tooltip("Main tabs")
      .Id("main-tabs")[+UI::Tab("Controls")
              .Open(isOpen)
              .Flags(ImGuiTabItemFlags_SetSelected)
              .Enabled(true)
              .Visible(true)
              .Tooltip("Controls")
              .Id("controls")[UI::Text("Controls")]];
});

int main() {
  std::vector<double> xValues{0.0, 1.0, 2.0};
  std::vector<double> yValues{0.0, 1.0, 4.0};
  const UI::DataView xView = UI::DataView::From(xValues);
  ds::RingBuffer<double> ringValues(3);

  ringValues.push_back(1.0);
  ringValues.push_back(2.0);
  ringValues.push_back(3.0);
  ringValues.push_back(4.0);

  assert(xView.GetData() == xValues.data());
  assert(xView.GetCount() == xValues.size());
  assert(xView.GetType() == UI::DataType::Double);
  assert(ringValues.capacity() == 3);
  assert(ringValues.size() == 3);
  assert(ringValues.offset() == 1);
  assert(ringValues[0] == 2.0);
  assert(ringValues[1] == 3.0);
  assert(ringValues[2] == 4.0);
  assert(ringValues.to_vector() == std::vector<double>({2.0, 3.0, 4.0}));

  const UI::DataView ringView = ringValues.data_view();
  assert(ringView.GetData() == ringValues.data());
  assert(ringView.GetCount() == ringValues.size());
  assert(ringView.GetType() == UI::DataType::Double);

  UI::UIElement text = UI::Text(std::string("Temporary text"));
  assert(text.IsValid());

  ImGui::CreateContext();
  ImPlot::CreateContext();

  ImGuiIO &io = ImGui::GetIO();
  io.DisplaySize = ImVec2(800.0F, 600.0F);
  io.DeltaTime = 1.0F / 60.0F;
  unsigned char *fontPixels = nullptr;
  int fontWidth = 0;
  int fontHeight = 0;
  io.Fonts->GetTexDataAsRGBA32(&fontPixels, &fontWidth, &fontHeight);

  bool isOpen = true;
  bool isTabOpen = true;
  bool isFoldOutOpen = true;
  bool enabled = false;
  double throttle = 0.5;
  int clicks = 0;

  ImGui::NewFrame();

  UI::Window("FlightUI Test")
      .Open(isOpen)
      .InitialSize({640.0F, 480.0F})[UI::VerticalLayout({
          UI::Heading("Controls"),
          UI::Text(std::string("Temporary text")),
          UI::FoldOut("Fold Out")
              .Open(isFoldOutOpen)
              .DefaultOpen()
              .Flags(ImGuiTreeNodeFlags_Framed)[UI::Text("Fold out body")],
          UI::Panel("Panel").Border(true)[UI::VerticalLayout({
              UI::Toggle("Enabled", enabled).OnChanged([&enabled](bool value) {
                enabled = value;
              }),
              UI::SliderDouble("Throttle", throttle, 0.0, 1.0)
                  .OnChanged([&throttle](double value) { throttle = value; })
                  .Width(180.0F),
              UI::ValueLabel("Throttle readout", throttle + 0.125, "{:.2f}"),
              UI::Button("Reset")
                  .OnAction([&clicks] { ++clicks; })
                  .Width(80.0F),
              UI::Custom([] { ImGui::TextUnformatted("Custom"); }),
          })],
          UI::TabGroup("Telemetry Tabs")
              .Flags(ImGuiTabBarFlags_Reorderable)
                  [+UI::Tab("Controls")[UI::Text("Control tab")]
                      + UI::Tab("Monitor").Open(isTabOpen).Tooltip(
                          "Monitor tab")[UI::Text("Monitor tab")]],
          UI::Plot("Plot")
              .Offset(1)
              .Height(120.0F)
              .XAxisFlags(ImPlotAxisFlags_None)
              .YAxisFlags(ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit)
              .XAxisLimits(0.0, 2.0, ImPlotCond_Always)
              .YAxisLimits(0.0, 4.0)
              .XAxisLabel("X")
              .YAxisLabel("Y")
              .AddLine("Line", xValues, yValues)
              .AddLine("Offset Line", xValues, yValues, 2)
              .AddLine("Ring Line", ringView, ringView, ringValues.offset())
              .AddScatter("Scatter", xValues, yValues, 1),
      })];

  // clang-format off
  UI::Window("Slate Style FlightUI Test")
  [
    +UI::Heading("Controls")
    + UI::Panel("Panel")
          .Border(true)
          [
            +UI::Toggle("Enabled", enabled)
                 .OnChanged([&enabled](bool value) { enabled = value; })
            + UI::SliderDouble("Throttle", throttle, 0.0, 1.0)
                  .OnChanged([&throttle](double value) { throttle = value; })
                  .Format("%.2f")
            + UI::Button("Reset").OnAction([&clicks] { ++clicks; })
          ]
    + UI::HorizontalLayout()
          .Spacing(8.0F)
          [
            +UI::Text("Left")
            + UI::Text("Right")
          ]
  ];
  // clang-format on

  UI::UIElement chainedLayout =
      UI::VerticalLayout() + UI::Text("First") + UI::Text("Second");
  assert(chainedLayout.IsValid());

  UI::Window("Second FlightUI Test")[UI::Text("Second window")];

  ImGui::Render();

  assert(isOpen);
  assert(isTabOpen);
  assert(isFoldOutOpen);
  assert(clicks == 0);

  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  return 0;
}
