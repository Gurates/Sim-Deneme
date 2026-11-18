package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCond;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import imgui.type.ImString;
import rsim2.mcp.McpConnectionSetup;
import rsim2.mcp.McpHttpEndpoint;

public class SceneObjectHierarchy {
    
    private final ImBoolean open = new ImBoolean(false);
    private final ImInt format = new ImInt(0);
    private final ImString preview = new ImString(32768);


    public void toggleVisible() {
        open.set(!open.get());
        open.get();
    }


    public void render() {
        if (!open.get()) return;
        ImGui.setNextWindowSize(690, 630, ImGuiCond.FirstUseEver);
        if (ImGui.begin("Scene Object Hierarchy", open)) {
            ImGui.textWrapped("Displays the hierarchy of scene objects in the current simulation.");
            ImGui.spacing();
        }
    }
}
