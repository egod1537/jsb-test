#include "application/gui/windows/LinearizationWindow.hpp"

#include "application/gui/GUI.hpp"
#include "application/sim/Simulation.hpp"
#include "application/sim/control/FlightControlManager.hpp"
#include "application/sim/gnc/Autopilot.hpp"
#include "application/sim/linearizer/LinearizationResult.hpp"
#include "flightui/core/Theme.hpp"
#include "flightui/core/UIScale.hpp"

#include <imgui.h>

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

namespace {
constexpr float MatrixCellWidth = 104.0F;
constexpr float MatrixRowLabelWidth = 112.0F;

std::string MakeLabel(const std::vector<std::string> &names, Eigen::Index index,
    const char *fallbackPrefix) {
  const auto nameIndex = static_cast<std::size_t>(index);
  if (nameIndex < names.size() && !names[nameIndex].empty()) {
    return names[nameIndex];
  }

  return std::string(fallbackPrefix) + std::to_string(index);
}

void DrawMatrix(const char *tableId, const Eigen::MatrixXd &matrix,
    const std::vector<std::string> &rowNames,
    const std::vector<std::string> &columnNames,
    const char *columnFallbackPrefix) {
  if (matrix.rows() == 0 || matrix.cols() == 0) {
    ImGui::TextDisabled("Matrix is empty.");
    return;
  }

  const int columnCount = static_cast<int>(matrix.cols()) + 1;
  constexpr ImGuiTableFlags Flags =
      ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollX
      | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingFixedFit
      | ImGuiTableFlags_Resizable;
  const float tableHeight =
      std::max(ImGui::GetContentRegionAvail().y, FlightUI::Ui(1.0F));
  if (!ImGui::BeginTable(tableId,
          columnCount,
          Flags,
          ImVec2(0.0F, tableHeight))) {
    return;
  }

  ImGui::TableSetupScrollFreeze(1, 1);
  ImGui::TableSetupColumn("d/dt",
      ImGuiTableColumnFlags_WidthFixed,
      FlightUI::Ui(MatrixRowLabelWidth));
  for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
    const std::string label =
        MakeLabel(columnNames, column, columnFallbackPrefix);
    ImGui::TableSetupColumn(label.c_str(),
        ImGuiTableColumnFlags_WidthFixed,
        FlightUI::Ui(MatrixCellWidth));
  }
  ImGui::TableHeadersRow();

  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    const std::string rowLabel = MakeLabel(rowNames, row, "x");
    ImGui::TextUnformatted(rowLabel.c_str());

    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      ImGui::TableSetColumnIndex(static_cast<int>(column) + 1);
      ImGui::Text("% .6e", matrix(row, column));
    }
  }

  ImGui::EndTable();
}
} // namespace

namespace gui {
LinearizationWindow::LinearizationWindow() : Window("FG Linearization") {}

void LinearizationWindow::OnRender(GUI &gui) {
  auto &simulation = gui.GetSimulation();
  auto *flightControlManager =
      simulation.GetComponent<control::FlightControlManager>();
  if (flightControlManager == nullptr) {
    ImGui::TextDisabled("Flight control manager is unavailable.");
    return;
  }

  const auto &autopilot = flightControlManager->GetAutopilot();
  const bool inProgress = autopilot.IsLinearizationInProgress();
  if (inProgress) {
    ImGui::TextDisabled("Updating asynchronously...");
  } else if (!autopilot.GetLinearizationErrorMessage().empty()) {
    ImGui::TextColored(
        FlightUI::GetDarkEditorSemanticColor(FlightUI::SemanticColor::Error),
        "Latest update failed: %.*s",
        static_cast<int>(autopilot.GetLinearizationErrorMessage().size()),
        autopilot.GetLinearizationErrorMessage().data());
  } else {
    ImGui::TextDisabled(autopilot.GetLinearizationResult() != nullptr
                            ? "Latest periodic result"
                            : "Waiting for periodic linearization...");
  }

  ImGui::Separator();
  const gnc::LinearizationResult *result = autopilot.GetLinearizationResult();
  if (result == nullptr) {
    ImGui::TextDisabled("No periodic result is available yet.");
    ImGui::TextDisabled(
        "Enable Roll Hold, Pitch Hold, or Course Hold in Autopilot mode.");
    return;
  }

  DrawResult(*result);
}

void LinearizationWindow::DrawResult(
    const gnc::LinearizationResult &result) const {
  ImGui::Text("A: %lld x %lld    B: %lld x %lld",
      static_cast<long long>(result.A.rows()),
      static_cast<long long>(result.A.cols()),
      static_cast<long long>(result.B.rows()),
      static_cast<long long>(result.B.cols()));

  if (!ImGui::BeginTabBar("LinearizationMatrices")) {
    return;
  }

  if (ImGui::BeginTabItem("A - System")) {
    DrawMatrix("SystemMatrix",
        result.A,
        result.stateNames,
        result.stateNames,
        "x");
    ImGui::EndTabItem();
  }

  if (ImGui::BeginTabItem("B - Input")) {
    DrawMatrix("InputMatrix",
        result.B,
        result.stateNames,
        result.inputNames,
        "u");
    ImGui::EndTabItem();
  }

  ImGui::EndTabBar();
}
} // namespace gui
