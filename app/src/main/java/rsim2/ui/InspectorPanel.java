package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiTreeNodeFlags;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImFloat;
import org.joml.Vector3f;
import rsim2.scene.SceneNode;

public class InspectorPanel {
    private SceneNode targetNode;
    private final ImFloat valX = new ImFloat();
    private final ImFloat valY = new ImFloat();
    private final ImFloat valZ = new ImFloat();

    public InspectorPanel(SceneNode targetNode) {
        this.targetNode = targetNode;
    }

    public void setTargetNode(SceneNode targetNode) {
        this.targetNode = targetNode;
    }

    public void render() {
        if (targetNode == null) return;

        float displayWidth = ImGui.getIO().getDisplaySizeX();
        float displayHeight = ImGui.getIO().getDisplaySizeY();
        float panelWidth = 300.0f;

        ImGui.setNextWindowPos(displayWidth - panelWidth, 0.0f, ImGuiCond.Always);
        ImGui.setNextWindowSize(panelWidth, displayHeight, ImGuiCond.Always);

        int flags = ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoCollapse;

        String windowTitle = "Robot Inspector (" + targetNode.getId() + ")";
        ImGui.begin(windowTitle, flags);

        Vector3f pos = targetNode.getLocalPosition();
        valX.set(pos.x);
        valY.set(pos.y);
        valZ.set(pos.z);

        if (ImGui.collapsingHeader("Position", ImGuiTreeNodeFlags.DefaultOpen)) {
            boolean changed = false;

            ImGui.text("X");
            ImGui.sameLine(30.0f);
            ImGui.setNextItemWidth(-1.0f);
            if (ImGui.inputFloat("##posX", valX, 0.1f, 1.0f, "%.2f")) {
                changed = true;
            }

            ImGui.text("Y");
            ImGui.sameLine(30.0f);
            ImGui.setNextItemWidth(-1.0f);
            if (ImGui.inputFloat("##posY", valY, 0.1f, 1.0f, "%.2f")) {
                changed = true;
            }

            ImGui.text("Z");
            ImGui.sameLine(30.0f);
            ImGui.setNextItemWidth(-1.0f);
            if (ImGui.inputFloat("##posZ", valZ, 0.1f, 1.0f, "%.2f")) {
                changed = true;
            }

            if (changed) {
                targetNode.getLocalPosition().set(valX.get(), valY.get(), valZ.get());
            }

            ImGui.spacing();
            if (ImGui.button("Reset Position", -1.0f, 28.0f)) {
                targetNode.getLocalPosition().set(0.0f, 0.5f, 0.0f);
            }
        }

        ImGui.end();
    }
}
