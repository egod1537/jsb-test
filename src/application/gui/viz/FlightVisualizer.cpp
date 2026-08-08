#include "application/gui/viz/FlightVisualizer.hpp"

#include "application/sim/Aircraft.hpp"
#include "application/gui/viz/components/AltitudeCueRenderer.hpp"
#include "application/gui/viz/components/AircraftWireframeRenderer.hpp"
#include "application/gui/viz/components/FlightCameraController.hpp"
#include "application/gui/viz/components/GroundGridRenderer.hpp"
#include "application/gui/viz/components/TelemetryOverlay.hpp"
#include "application/gui/viz/render/CameraComponent.hpp"
#include "application/gui/viz/render/LineCanvas.hpp"
#include "flightui/core/UIScale.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>

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
    ImGui::EndPopup();
  }

  ImGui::PopID();
}

void FlightVisualizer::ToggleViewMode() {
  viewMode_ =
      viewMode_ == ViewMode::Orbit ? ViewMode::ThirdPerson : ViewMode::Orbit;
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
  const float headingRad = static_cast<float>(state.headingDeg) * DegToRad;
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
