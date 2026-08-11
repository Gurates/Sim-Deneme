package rsim2.ui;

import imgui.ImGui;
import imgui.ImGuiStyle;
import imgui.flag.ImGuiCol;
import imgui.gl3.ImGuiImplGl3;
import imgui.glfw.ImGuiImplGlfw;

public class ImGuiLayer {
    private final ImGuiImplGlfw imguiGlfw = new ImGuiImplGlfw();
    private final ImGuiImplGl3 imguiGl3 = new ImGuiImplGl3();

    public void init(long windowHandle) {
        ImGui.createContext();

        applyCustomStyle();

        imguiGlfw.init(windowHandle, true);
        imguiGl3.init("#version 330");
    }

    private void applyCustomStyle() {
        ImGui.styleColorsLight();

        ImGuiStyle style = ImGui.getStyle();

        style.setWindowPadding(12.0f, 12.0f);
        style.setFramePadding(8.0f, 6.0f);
        style.setItemSpacing(8.0f, 8.0f);
        style.setItemInnerSpacing(6.0f, 6.0f);

        style.setWindowRounding(4.0f);
        style.setFrameRounding(4.0f);
        style.setGrabRounding(4.0f);
        style.setPopupRounding(4.0f);
        style.setScrollbarRounding(4.0f);

        style.setWindowBorderSize(1.0f);
        style.setFrameBorderSize(1.0f);
        style.setPopupBorderSize(1.0f);

        style.setColor(ImGuiCol.WindowBg, 0.94f, 0.94f, 0.95f, 1.00f);
        style.setColor(ImGuiCol.ChildBg, 0.96f, 0.96f, 0.97f, 1.00f);
        style.setColor(ImGuiCol.PopupBg, 0.98f, 0.98f, 0.98f, 0.98f);
        style.setColor(ImGuiCol.Border, 0.78f, 0.78f, 0.80f, 0.70f);
        style.setColor(ImGuiCol.BorderShadow, 0.00f, 0.00f, 0.00f, 0.00f);

        style.setColor(ImGuiCol.FrameBg, 1.00f, 1.00f, 1.00f, 1.00f);
        style.setColor(ImGuiCol.FrameBgHovered, 0.92f, 0.95f, 0.98f, 1.00f);
        style.setColor(ImGuiCol.FrameBgActive, 0.85f, 0.90f, 0.96f, 1.00f);

        style.setColor(ImGuiCol.TitleBg, 0.90f, 0.90f, 0.92f, 1.00f);
        style.setColor(ImGuiCol.TitleBgActive, 0.86f, 0.88f, 0.92f, 1.00f);
        style.setColor(ImGuiCol.TitleBgCollapsed, 0.94f, 0.94f, 0.94f, 0.75f);

        style.setColor(ImGuiCol.MenuBarBg, 0.92f, 0.92f, 0.94f, 1.00f);
        style.setColor(ImGuiCol.ScrollbarBg, 0.94f, 0.94f, 0.95f, 0.60f);
        style.setColor(ImGuiCol.ScrollbarGrab, 0.75f, 0.75f, 0.78f, 0.80f);
        style.setColor(ImGuiCol.ScrollbarGrabHovered, 0.65f, 0.65f, 0.68f, 0.90f);
        style.setColor(ImGuiCol.ScrollbarGrabActive, 0.55f, 0.55f, 0.58f, 1.00f);

        style.setColor(ImGuiCol.CheckMark, 0.20f, 0.45f, 0.80f, 1.00f);
        style.setColor(ImGuiCol.SliderGrab, 0.20f, 0.45f, 0.80f, 1.00f);
        style.setColor(ImGuiCol.SliderGrabActive, 0.15f, 0.38f, 0.75f, 1.00f);

        style.setColor(ImGuiCol.Button, 0.88f, 0.88f, 0.90f, 1.00f);
        style.setColor(ImGuiCol.ButtonHovered, 0.80f, 0.84f, 0.90f, 1.00f);
        style.setColor(ImGuiCol.ButtonActive, 0.72f, 0.78f, 0.86f, 1.00f);

        style.setColor(ImGuiCol.Header, 0.88f, 0.90f, 0.94f, 1.00f);
        style.setColor(ImGuiCol.HeaderHovered, 0.82f, 0.86f, 0.92f, 1.00f);
        style.setColor(ImGuiCol.HeaderActive, 0.75f, 0.80f, 0.88f, 1.00f);

        style.setColor(ImGuiCol.Separator, 0.78f, 0.78f, 0.80f, 0.60f);
        style.setColor(ImGuiCol.SeparatorHovered, 0.60f, 0.65f, 0.75f, 0.80f);
        style.setColor(ImGuiCol.SeparatorActive, 0.40f, 0.50f, 0.70f, 1.00f);

        style.setColor(ImGuiCol.Text, 0.15f, 0.15f, 0.17f, 1.00f);
        style.setColor(ImGuiCol.TextDisabled, 0.50f, 0.50f, 0.52f, 1.00f);
    }

    public void newFrame() {
        imguiGlfw.newFrame();
        imguiGl3.newFrame();
        ImGui.newFrame();
    }

    public void render() {
        ImGui.render();
        imguiGl3.renderDrawData(ImGui.getDrawData());
    }

    public void dispose() {
        ImGui.destroyContext();
    }
}
