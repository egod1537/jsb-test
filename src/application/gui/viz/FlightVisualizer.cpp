#include "application/gui/viz/FlightVisualizer.hpp"

#include "application/sim/Aircraft.hpp"
#include "application/gui/viz/components/AltitudeCueRenderer.hpp"
#include "application/gui/viz/components/AircraftWireframeRenderer.hpp"
#include "application/gui/viz/components/FlightCameraController.hpp"
#include "application/gui/viz/components/GroundGridRenderer.hpp"
#include "application/gui/viz/components/TelemetryOverlay.hpp"
#include "application/gui/viz/render/CameraComponent.hpp"
#include "application/gui/viz/render/LineCanvas.hpp"
#include "common/math/Math.hpp"
#include "flightui/core/UIScale.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {
constexpr float MinVisualAltitude = 0.35F;
constexpr float MaxVisualAltitude = 52.0F;
constexpr float LinearAltitudeBreakFt = 1800.0F;
constexpr float FeetPerVizUnit = 75.0F;
constexpr float HighAltitudeLogFt = 450.0F;
constexpr float HighAltitudeLogScale = 8.0F;
constexpr float MetersPerVizUnit = FeetPerVizUnit * 0.3048F;
constexpr double MaxMotionTickSec = 0.25;
constexpr double KnotsToMetersPerSec = 0.5144444444444445;
constexpr double MinimumMinimapSpanMeters = 100.0;

float VisualAltitudeFromAglFt(double altitudeAglFt) {
  if (!std::isfinite(altitudeAglFt)) {
    return MinVisualAltitude;
  }

  const float altitudeFt = static_cast<float>(std::max(altitudeAglFt, 0.0));
  if (altitudeFt <= LinearAltitudeBreakFt) {
    return std::clamp(altitudeFt / FeetPerVizUnit,
        MinVisualAltitude,
        MaxVisualAltitude);
  }

  const float linearAltitude = LinearAltitudeBreakFt / FeetPerVizUnit;
  const float compressedAltitude =
      linearAltitude
      + std::log1p((altitudeFt - LinearAltitudeBreakFt) / HighAltitudeLogFt)
            * HighAltitudeLogScale;
  return std::clamp(compressedAltitude, MinVisualAltitude, MaxVisualAltitude);
}

float HorizontalSpeedMps(const sim::AircraftState &state) {
  if (std::isfinite(state.trueAirspeedMps) && state.trueAirspeedMps > 0.1) {
    return static_cast<float>(state.trueAirspeedMps);
  }

  if (std::isfinite(state.calibratedAirspeedKts)
      && state.calibratedAirspeedKts > 0.1) {
    return static_cast<float>(
        state.calibratedAirspeedKts * KnotsToMetersPerSec);
  }

  return 0.0F;
}
} // namespace

namespace viz {
FlightVisualizer::FlightVisualizer() { BuildScene(); }

FlightVisualizer::~FlightVisualizer() = default;

bool FlightVisualizer::Tick(const sim::Aircraft &aircraft) {
  snapshot_.aircraftState = aircraft.GetAircraftState();
  snapshot_.controlInput = aircraft.GetControls().GetInput();
  snapshot_.pitchTrim = aircraft.GetControls().GetPitchTrim();
  SyncFlightPath(aircraft);
  SyncGroundScroll(snapshot_.aircraftState);
  snapshot_.viewMode = viewMode_;
  snapshot_.viewOptions = viewOptions_;
  snapshot_.groundScroll = groundScroll_;
  snapshot_.visualAltitude =
      VisualAltitudeFromAglFt(snapshot_.aircraftState.altitudeAglFt);
  snapshot_.hasAircraft = true;
  scene_.Tick(snapshot_);
  return true;
}

void FlightVisualizer::RenderScene() {
  if (!snapshot_.hasAircraft) {
    ImGui::TextDisabled("No visualization snapshot.");
    return;
  }

  HandleInput();
  snapshot_.viewMode = viewMode_;
  snapshot_.viewOptions = viewOptions_;
  snapshot_.groundScroll = groundScroll_;
  scene_.Tick(snapshot_);

  const ImVec2 available = ImGui::GetContentRegionAvail();
  const ImVec2 size{
      std::max(available.x, 1.0F),
      std::max(available.y, 1.0F),
  };

  ImGui::SetNextItemAllowOverlap();
  ImGui::InvisibleButton("##FlightVizCanvas", size);
  const ImVec2 min = ImGui::GetItemRectMin();
  const ImVec2 max = ImGui::GetItemRectMax();
  const float focalLength = std::min(size.x, size.y) * 0.82F;

  ImDrawList *drawList = ImGui::GetWindowDrawList();
  drawList->PushClipRect(min, max, true);

  const CameraView camera =
      mainCamera_ != nullptr ? mainCamera_->BuildView() : CameraView{};
  LineCanvas canvas(*drawList, min, max, camera, focalLength);
  canvas.Fill(IM_COL32(13, 16, 21, 255));

  RenderContext context{snapshot_, canvas};
  scene_.Render(context);
  RenderMinimap(min, max);

  canvas.Border(IM_COL32(88, 96, 108, 255));
  drawList->PopClipRect();

  RenderViewOptionsMenu(min, max);
  snapshot_.viewOptions = viewOptions_;
}

void FlightVisualizer::RenderAircraftWireframe() { RenderScene(); }

void FlightVisualizer::HandleInput() {
  const ImGuiIO &io = ImGui::GetIO();
  if (io.WantTextInput
      || !ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)) {
    return;
  }

  if (ImGui::IsKeyPressed(ImGuiKey_V, false)) {
    ToggleViewMode();
  }
}

void FlightVisualizer::RenderViewOptionsMenu(ImVec2 min, ImVec2 max) {
  constexpr float Padding = 10.0F;
  constexpr float ButtonWidth = 64.0F;

  ImGui::PushID("FlightVizViewOptions");
  ImGui::SetCursorScreenPos(
      ImVec2(max.x - FlightUI::Ui(ButtonWidth) - FlightUI::Ui(Padding),
          min.y + FlightUI::Ui(Padding)));

  if (ImGui::Button("View", ImVec2(FlightUI::Ui(ButtonWidth), 0.0F))) {
    ImGui::OpenPopup("ViewOptions");
  }

  if (ImGui::BeginPopup("ViewOptions")) {
    ImGui::Checkbox("Ground Grid", &viewOptions_.showGroundGrid);
    ImGui::Checkbox("Telemetry", &viewOptions_.showTelemetry);
    ImGui::Checkbox("Minimap", &viewOptions_.showMinimap);
    if (viewOptions_.showMinimap && ImGui::Button("Clear Minimap Path")) {
      flightPathHistory_.Reset();
    }
    ImGui::EndPopup();
  }

  ImGui::PopID();
}

void FlightVisualizer::ToggleViewMode() {
  viewMode_ =
      viewMode_ == ViewMode::Orbit ? ViewMode::ThirdPerson : ViewMode::Orbit;
}

void FlightVisualizer::RenderMinimap(ImVec2 min, ImVec2 max) {
  if (!viewOptions_.showMinimap) {
    return;
  }

  const auto &points = flightPathHistory_.GetPoints();
  const std::optional<FlightPathPoint> currentPoint =
      flightPathHistory_.GetCurrentPoint();
  if (points.empty() || !currentPoint.has_value()) {
    return;
  }

  const float canvasWidth = std::max(max.x - min.x, 1.0F);
  const float canvasHeight = std::max(max.y - min.y, 1.0F);
  const float maximumSize =
      std::max(std::min(canvasWidth * 0.34F, canvasHeight * 0.38F), 1.0F);
  const float minimapSize = std::min(FlightUI::Ui(210.0F), maximumSize);
  const float outerPadding = std::min(FlightUI::Ui(10.0F), minimapSize * 0.08F);
  const ImVec2 mapMax{
      max.x - outerPadding,
      max.y - outerPadding,
  };
  const ImVec2 mapMin{
      mapMax.x - minimapSize,
      mapMax.y - minimapSize,
  };

  ImDrawList &drawList = *ImGui::GetWindowDrawList();
  drawList.AddRectFilled(mapMin,
      mapMax,
      IM_COL32(30, 30, 30, 224),
      FlightUI::Ui(3.0F));
  drawList.AddRect(mapMin,
      mapMax,
      IM_COL32(76, 82, 90, 255),
      FlightUI::Ui(3.0F),
      0,
      FlightUI::Ui(1.0F));

  const float headerHeight = std::min(FlightUI::Ui(24.0F), minimapSize * 0.18F);
  const float contentPadding =
      std::min(FlightUI::Ui(10.0F), minimapSize * 0.06F);
  const ImVec2 plotMin{
      mapMin.x + contentPadding,
      mapMin.y + headerHeight,
  };
  const ImVec2 plotMax{
      mapMax.x - contentPadding,
      mapMax.y - contentPadding,
  };

  double minimumNorth = currentPoint->northMeters;
  double maximumNorth = currentPoint->northMeters;
  double minimumEast = currentPoint->eastMeters;
  double maximumEast = currentPoint->eastMeters;
  for (const FlightPathPoint &point : points) {
    minimumNorth = std::min(minimumNorth, point.northMeters);
    maximumNorth = std::max(maximumNorth, point.northMeters);
    minimumEast = std::min(minimumEast, point.eastMeters);
    maximumEast = std::max(maximumEast, point.eastMeters);
  }

  const double centerNorth = (minimumNorth + maximumNorth) * 0.5;
  const double centerEast = (minimumEast + maximumEast) * 0.5;
  const double spanMeters = std::max({maximumNorth - minimumNorth,
      maximumEast - minimumEast,
      MinimumMinimapSpanMeters});
  const float plotWidth = std::max(plotMax.x - plotMin.x, 1.0F);
  const float plotHeight = std::max(plotMax.y - plotMin.y, 1.0F);
  const float pixelsPerMeter =
      std::min(plotWidth, plotHeight) / static_cast<float>(spanMeters * 1.15);
  const ImVec2 plotCenter{
      (plotMin.x + plotMax.x) * 0.5F,
      (plotMin.y + plotMax.y) * 0.5F,
  };

  const auto projectPoint = [&](const FlightPathPoint &point) {
    return ImVec2(plotCenter.x
                      + static_cast<float>(point.eastMeters - centerEast)
                            * pixelsPerMeter,
        plotCenter.y
            - static_cast<float>(point.northMeters - centerNorth)
                  * pixelsPerMeter);
  };

  char title[64]{};
  std::snprintf(title, sizeof(title), "PATH  %.0f m", spanMeters);
  drawList.AddText(
      ImVec2(mapMin.x + contentPadding, mapMin.y + FlightUI::Ui(5.0F)),
      IM_COL32(214, 214, 214, 255),
      title);
  drawList.AddText(ImVec2(mapMax.x - contentPadding - FlightUI::Ui(9.0F),
                       mapMin.y + FlightUI::Ui(5.0F)),
      IM_COL32(128, 156, 182, 255),
      "N");

  drawList.PushClipRect(plotMin, plotMax, true);
  drawList.AddLine(ImVec2(plotMin.x, plotCenter.y),
      ImVec2(plotMax.x, plotCenter.y),
      IM_COL32(63, 63, 63, 180),
      FlightUI::Ui(1.0F));
  drawList.AddLine(ImVec2(plotCenter.x, plotMin.y),
      ImVec2(plotCenter.x, plotMax.y),
      IM_COL32(63, 63, 63, 180),
      FlightUI::Ui(1.0F));

  auto pointIterator = points.begin();
  ImVec2 previousScreenPoint = projectPoint(*pointIterator);
  const ImVec2 startScreenPoint = previousScreenPoint;
  ++pointIterator;
  for (; pointIterator != points.end(); ++pointIterator) {
    const ImVec2 screenPoint = projectPoint(*pointIterator);
    drawList.AddLine(previousScreenPoint,
        screenPoint,
        IM_COL32(83, 151, 211, 255),
        FlightUI::Ui(2.0F));
    previousScreenPoint = screenPoint;
  }
  const ImVec2 currentScreenPoint = projectPoint(*currentPoint);
  drawList.AddLine(previousScreenPoint,
      currentScreenPoint,
      IM_COL32(83, 151, 211, 255),
      FlightUI::Ui(2.0F));
  drawList.AddCircleFilled(startScreenPoint,
      FlightUI::Ui(3.0F),
      IM_COL32(107, 166, 112, 255));

  const double courseRad = math::DegToRad(snapshot_.aircraftState.courseDeg);
  const ImVec2 forward{
      static_cast<float>(std::sin(courseRad)),
      static_cast<float>(-std::cos(courseRad)),
  };
  const ImVec2 right{-forward.y, forward.x};
  const float markerLength = FlightUI::Ui(9.0F);
  const float markerHalfWidth = FlightUI::Ui(5.0F);
  const ImVec2 markerTip{
      currentScreenPoint.x + forward.x * markerLength,
      currentScreenPoint.y + forward.y * markerLength,
  };
  const ImVec2 markerLeft{
      currentScreenPoint.x - forward.x * markerLength * 0.55F
          + right.x * markerHalfWidth,
      currentScreenPoint.y - forward.y * markerLength * 0.55F
          + right.y * markerHalfWidth,
  };
  const ImVec2 markerRight{
      currentScreenPoint.x - forward.x * markerLength * 0.55F
          - right.x * markerHalfWidth,
      currentScreenPoint.y - forward.y * markerLength * 0.55F
          - right.y * markerHalfWidth,
  };
  drawList.AddTriangleFilled(markerTip,
      markerLeft,
      markerRight,
      IM_COL32(230, 235, 240, 255));
  drawList.PopClipRect();
}

void FlightVisualizer::SyncFlightPath(const sim::Aircraft &aircraft) {
  const auto &properties = aircraft.GetProperties();
  flightPathHistory_.AddSample(snapshot_.aircraftState.simulationTimeSec,
      properties.Latitude().Rad(),
      properties.Longitude().Rad());
}

void FlightVisualizer::SyncGroundScroll(const sim::AircraftState &state) {
  const double sampleTime = state.simulationTimeSec;
  if (!std::isfinite(sampleTime)) {
    hasMotionSample_ = false;
    return;
  }

  if (!hasMotionSample_ || sampleTime < lastMotionSampleTimeSec_) {
    lastMotionSampleTimeSec_ = sampleTime;
    hasMotionSample_ = true;
    groundScroll_ = {};
    return;
  }

  const double dt = sampleTime - lastMotionSampleTimeSec_;
  lastMotionSampleTimeSec_ = sampleTime;
  if (dt <= 0.0 || dt > MaxMotionTickSec) {
    return;
  }

  const float speedMps = HorizontalSpeedMps(state);
  if (speedMps <= 0.0F || !std::isfinite(state.headingDeg)) {
    return;
  }

  const float distanceViz =
      static_cast<float>(dt) * speedMps / MetersPerVizUnit;
  const float headingRad = static_cast<float>(math::DegToRad(state.headingDeg));
  const Vec3 forward{std::cos(headingRad), std::sin(headingRad), 0.0F};

  groundScroll_ = groundScroll_ - forward * distanceViz;
}

void FlightVisualizer::BuildScene() {
  scene_.Clear();

  GameObject &cameraObject = scene_.CreateGameObject("Main Camera");
  mainCamera_ = &cameraObject.AddComponent<CameraComponent>();
  mainCamera_->SetEye({5.5F, -8.0F, 4.2F});
  mainCamera_->SetTarget({0.0F, 0.0F, 0.2F});
  mainCamera_->SetWorldUp({0.0F, 0.0F, 1.0F});
  FlightCameraController &cameraController =
      cameraObject.AddComponent<FlightCameraController>();
  cameraController.SetCamera(mainCamera_);

  GameObject &groundGrid = scene_.CreateGameObject("Ground Grid");
  groundGrid.GetTransform().SetPosition({0.0F, 0.0F, -0.9F});
  groundGrid.AddComponent<GroundGridRenderer>();

  GameObject &altitudeCue = scene_.CreateGameObject("Altitude Cue");
  altitudeCue.AddComponent<AltitudeCueRenderer>();

  GameObject &aircraft = scene_.CreateGameObject("Aircraft");
  aircraft.AddComponent<AircraftWireframeRenderer>();

  GameObject &overlay = scene_.CreateGameObject("Telemetry Overlay");
  overlay.AddComponent<TelemetryOverlay>();
}
} // namespace viz
