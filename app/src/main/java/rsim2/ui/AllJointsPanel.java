package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiWindowFlags;
import rsim2.core.Engine;
import rsim2.editor.commands.CommandHistory;
import rsim2.editor.commands.DeleteJointCommand;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;

import java.util.ArrayList;
import java.util.List;

public class AllJointsPanel {
    private final Engine engine;
    private SceneNode rootNode;
    private List<Joint> allJoints;
    private CommandHistory commandHistory;

    public AllJointsPanel(Engine engine, SceneNode rootNode, List<Joint> allJoints, CommandHistory commandHistory) {
        this.engine = engine;
        this.rootNode = rootNode;
        this.allJoints = allJoints != null ? allJoints : new ArrayList<>();
        this.commandHistory = commandHistory;
    }

    public void setRootNode(SceneNode rootNode) {
        this.rootNode = rootNode;
    }

    public void setJoints(List<Joint> allJoints) {
        this.allJoints = allJoints != null ? allJoints : new ArrayList<>();
    }

    public void setCommandHistory(CommandHistory commandHistory) {
        this.commandHistory = commandHistory;
    }

    public void render() {
        float displayHeight = ImGui.getIO().getDisplaySizeY();
        float panelWidth = 280.0f;
        float topOffsetY = 40.0f;
        float totalHeight = displayHeight - topOffsetY;
        float hierarchyHeight = totalHeight * 0.5f;
        float panelPosY = topOffsetY + hierarchyHeight;
        float panelHeight = totalHeight - hierarchyHeight;

        ImGui.setNextWindowPos(0.0f, panelPosY, ImGuiCond.Always);
        ImGui.setNextWindowSize(panelWidth, panelHeight, ImGuiCond.Always);

        int flags = ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoCollapse;

        ImGui.begin("All Joints", flags);

        ImGui.text("Configured Joints (" + allJoints.size() + ")");
        ImGui.spacing();

        ImGui.beginChild("##allJointsList", 0.0f, 0.0f, true);

        Joint jointToDelete = null;

        if (allJoints.isEmpty()) {
            ImGui.textDisabled("No joints in project.\nDrag a parent to 'Connected Body'\nin Inspector to connect parts.");
        } else {
            for (Joint joint : allJoints) {
                String pName = joint.getParentNode() != null ? joint.getParentNode().getId() : "null";
                String cName = joint.getChildNode() != null ? joint.getChildNode().getId() : "null";
                String typeStr = joint.getType() != null ? joint.getType().name() : "REVOLUTE";

                ImGui.text(pName + " -> " + cName);
                ImGui.textDisabled("(" + typeStr + ")");
                ImGui.sameLine(ImGui.getWindowWidth() - 75.0f);

                ImGui.pushStyleColor(ImGuiCol.Button, 0.75f, 0.2f, 0.2f, 1.0f);
                ImGui.pushStyleColor(ImGuiCol.ButtonHovered, 0.85f, 0.3f, 0.3f, 1.0f);
                ImGui.pushStyleColor(ImGuiCol.ButtonActive, 0.65f, 0.1f, 0.1f, 1.0f);
                if (ImGui.button("Delete##del_" + joint.getId(), 60.0f, 22.0f)) {
                    jointToDelete = joint;
                }
                ImGui.popStyleColor(3);

                ImGui.separator();
            }
        }

        ImGui.endChild();

        if (jointToDelete != null) {
            deleteJoint(jointToDelete);
        }

        ImGui.end();
    }

    private void deleteJoint(Joint joint) {
        if (joint == null) return;

        if (commandHistory != null) {
            commandHistory.executeAndRecord(new DeleteJointCommand(joint, allJoints, rootNode));
        } else {
            SceneNode childNode = joint.getChildNode();
            if (childNode != null) {
                childNode.reparentPreservingWorldTransform(rootNode);
            }
            allJoints.remove(joint);
            if (engine != null) {
                engine.autoSaveProject();
            }
        }
    }
}
