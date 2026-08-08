#include "flightui/core/Theme.hpp"

#include <cstdint>
#include <imgui.h>
#include <implot.h>

namespace FlightUI {
namespace {
ImVec4 Color(std::uint32_t rgb, float alpha = 1.0F) {
  constexpr float ByteToFloat = 1.0F / 255.0F;
  return {
      static_cast<float>((rgb >> 16U) & 0xFFU) * ByteToFloat,
      static_cast<float>((rgb >> 8U) & 0xFFU) * ByteToFloat,
      static_cast<float>(rgb & 0xFFU) * ByteToFloat,
      alpha,
  };
}

struct DarkEditorPalette {
  ImVec4 applicationBackground = Color(0x1E1E1E);
  ImVec4 windowBackground = Color(0x252525);
  ImVec4 childBackground = Color(0x2A2A2A);
  ImVec4 frameBackground = Color(0x333333);
  ImVec4 frameHovered = Color(0x3D3D3D);
  ImVec4 frameActive = Color(0x454545);
  ImVec4 border = Color(0x3F3F3F);
  ImVec4 separator = Color(0x404040);
  ImVec4 text = Color(0xD6D6D6);
  ImVec4 textDisabled = Color(0x808080);
  ImVec4 accent = Color(0x3A7EBF);
  ImVec4 accentHovered = Color(0x478DCE);
  ImVec4 accentActive = Color(0x2F6FAE);
  ImVec4 success = Color(0x609B6D);
  ImVec4 warning = Color(0xBC8848);
  ImVec4 error = Color(0xB85F5F);
};

const DarkEditorPalette &Palette() {
  static const DarkEditorPalette palette;
  return palette;
}

void ApplyImGuiTheme(const DarkEditorPalette &palette) {
  ImGuiStyle &style = ImGui::GetStyle();
  ImGui::StyleColorsDark(&style);

  style.WindowPadding = ImVec2(8.0F, 8.0F);
  style.FramePadding = ImVec2(4.0F, 3.0F);
  style.ItemSpacing = ImVec2(8.0F, 4.0F);
  style.ItemInnerSpacing = ImVec2(4.0F, 4.0F);
  style.CellPadding = ImVec2(4.0F, 2.0F);
  style.IndentSpacing = 21.0F;
  style.ScrollbarSize = 13.0F;
  style.GrabMinSize = 11.0F;

  style.WindowRounding = 3.0F;
  style.ChildRounding = 3.0F;
  style.PopupRounding = 3.0F;
  style.FrameRounding = 2.0F;
  style.ScrollbarRounding = 3.0F;
  style.GrabRounding = 2.0F;
  style.TabRounding = 3.0F;
  style.MenuItemRounding = 2.0F;
  style.SelectableRounding = 2.0F;

  style.WindowBorderSize = 1.0F;
  style.ChildBorderSize = 1.0F;
  style.PopupBorderSize = 1.0F;
  style.FrameBorderSize = 1.0F;
  style.TabBorderSize = 0.0F;
  style.TabBarBorderSize = 1.0F;
  style.TabBarOverlineSize = 2.0F;
  style.SeparatorSize = 1.0F;
  style.DockingSeparatorSize = 1.0F;

  ImVec4 *colors = style.Colors;
  colors[ImGuiCol_Text] = palette.text;
  colors[ImGuiCol_TextDisabled] = palette.textDisabled;
  colors[ImGuiCol_WindowBg] = palette.windowBackground;
  colors[ImGuiCol_ChildBg] = palette.childBackground;
  colors[ImGuiCol_PopupBg] = Color(0x292929, 0.98F);
  colors[ImGuiCol_Border] = palette.border;
  colors[ImGuiCol_BorderShadow] = Color(0x000000, 0.18F);
  colors[ImGuiCol_FrameBg] = palette.frameBackground;
  colors[ImGuiCol_FrameBgHovered] = palette.frameHovered;
  colors[ImGuiCol_FrameBgActive] = palette.frameActive;
  colors[ImGuiCol_TitleBg] = Color(0x202020);
  colors[ImGuiCol_TitleBgActive] = Color(0x292929);
  colors[ImGuiCol_TitleBgCollapsed] = Color(0x202020);
  colors[ImGuiCol_MenuBarBg] = Color(0x222222);
  colors[ImGuiCol_ScrollbarBg] = Color(0x202020, 0.75F);
  colors[ImGuiCol_ScrollbarGrab] = Color(0x4A4A4A);
  colors[ImGuiCol_ScrollbarGrabHovered] = Color(0x595959);
  colors[ImGuiCol_ScrollbarGrabActive] = Color(0x666666);
  colors[ImGuiCol_CheckMark] = palette.accentHovered;
  colors[ImGuiCol_CheckboxSelectedBg] = Color(0x315E86);
  colors[ImGuiCol_SliderGrab] = palette.accent;
  colors[ImGuiCol_SliderGrabActive] = palette.accentHovered;
  colors[ImGuiCol_Button] = palette.frameBackground;
  colors[ImGuiCol_ButtonHovered] = palette.frameHovered;
  colors[ImGuiCol_ButtonActive] = Color(0x384E61);
  colors[ImGuiCol_Header] = Color(0x33414B);
  colors[ImGuiCol_HeaderHovered] = Color(0x3C4D59);
  colors[ImGuiCol_HeaderActive] = Color(0x315E82);
  colors[ImGuiCol_Separator] = palette.separator;
  colors[ImGuiCol_SeparatorHovered] = Color(0x3A6F9B);
  colors[ImGuiCol_SeparatorActive] = palette.accent;
  colors[ImGuiCol_ResizeGrip] = Color(0x3A7EBF, 0.22F);
  colors[ImGuiCol_ResizeGripHovered] = Color(0x478DCE, 0.62F);
  colors[ImGuiCol_ResizeGripActive] = palette.accent;
  colors[ImGuiCol_InputTextCursor] = palette.text;
  colors[ImGuiCol_TabHovered] = Color(0x3A4853);
  colors[ImGuiCol_Tab] = Color(0x2C2C2C);
  colors[ImGuiCol_TabSelected] = Color(0x363E45);
  colors[ImGuiCol_TabSelectedOverline] = palette.accent;
  colors[ImGuiCol_TabDimmed] = Color(0x252525);
  colors[ImGuiCol_TabDimmedSelected] = Color(0x30363B);
  colors[ImGuiCol_TabDimmedSelectedOverline] = Color(0x376B99);
  colors[ImGuiCol_DockingPreview] = Color(0x3A7EBF, 0.55F);
  colors[ImGuiCol_DockingEmptyBg] = palette.applicationBackground;
  colors[ImGuiCol_PlotLines] = palette.accentHovered;
  colors[ImGuiCol_PlotLinesHovered] = Color(0x70A9DB);
  colors[ImGuiCol_PlotHistogram] = palette.warning;
  colors[ImGuiCol_PlotHistogramHovered] = Color(0xD19B58);
  colors[ImGuiCol_TableHeaderBg] = Color(0x303030);
  colors[ImGuiCol_TableBorderStrong] = palette.border;
  colors[ImGuiCol_TableBorderLight] = Color(0x393939);
  colors[ImGuiCol_TableRowBg] = Color(0x000000, 0.0F);
  colors[ImGuiCol_TableRowBgAlt] = Color(0x363636, 0.32F);
  colors[ImGuiCol_TextLink] = palette.accentHovered;
  colors[ImGuiCol_TextSelectedBg] = Color(0x3A7EBF, 0.42F);
  colors[ImGuiCol_TreeLines] = Color(0x4A4A4A);
  colors[ImGuiCol_DragDropTarget] = palette.accentHovered;
  colors[ImGuiCol_DragDropTargetBg] = Color(0x3A7EBF, 0.18F);
  colors[ImGuiCol_UnsavedMarker] = palette.warning;
  colors[ImGuiCol_NavCursor] = palette.accentHovered;
  colors[ImGuiCol_NavWindowingHighlight] = Color(0xD6D6D6, 0.65F);
  colors[ImGuiCol_NavWindowingDimBg] = Color(0x101010, 0.52F);
  colors[ImGuiCol_ModalWindowDimBg] = Color(0x101010, 0.62F);
}

void ApplyImPlotTheme(const DarkEditorPalette &palette) {
  ImPlotStyle &style = ImPlot::GetStyle();
  ImPlot::StyleColorsDark(&style);

  style.PlotBorderSize = 1.0F;
  style.MinorAlpha = 0.30F;
  style.Colormap = ImPlotColormap_Deep;

  ImVec4 *colors = style.Colors;
  colors[ImPlotCol_FrameBg] = palette.childBackground;
  colors[ImPlotCol_PlotBg] = Color(0x202020);
  colors[ImPlotCol_PlotBorder] = palette.border;
  colors[ImPlotCol_LegendBg] = Color(0x252525, 0.96F);
  colors[ImPlotCol_LegendBorder] = palette.border;
  colors[ImPlotCol_LegendText] = palette.text;
  colors[ImPlotCol_TitleText] = palette.text;
  colors[ImPlotCol_InlayText] = palette.text;
  colors[ImPlotCol_AxisText] = Color(0xA8A8A8);
  colors[ImPlotCol_AxisGrid] = Color(0x777777, 0.22F);
  colors[ImPlotCol_AxisTick] = Color(0x888888, 0.48F);
  colors[ImPlotCol_AxisBg] = Color(0x000000, 0.0F);
  colors[ImPlotCol_AxisBgHovered] = Color(0x3D3D3D, 0.70F);
  colors[ImPlotCol_AxisBgActive] = Color(0x3A7EBF, 0.34F);
  colors[ImPlotCol_Selection] = palette.accentHovered;
  colors[ImPlotCol_Crosshairs] = Color(0xA0A0A0, 0.72F);
}
} // namespace

void ApplyDarkEditorTheme() {
  const DarkEditorPalette &palette = Palette();
  ApplyImGuiTheme(palette);
  ApplyImPlotTheme(palette);
}

ImVec4 GetDarkEditorApplicationBackground() {
  return Palette().applicationBackground;
}

ImVec4 GetDarkEditorSemanticColor(SemanticColor color) {
  const DarkEditorPalette &palette = Palette();
  switch (color) {
  case SemanticColor::Success:
    return palette.success;
  case SemanticColor::Warning:
    return palette.warning;
  case SemanticColor::Error:
    return palette.error;
  }

  return palette.textDisabled;
}
} // namespace FlightUI
