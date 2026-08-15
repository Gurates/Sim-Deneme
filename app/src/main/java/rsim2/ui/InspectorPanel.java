package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiTreeNodeFlags;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImFloat;
import org.joml.Vector3f;
import rsim2.editor.SelectionManager;
import rsim2.scene.Joint;
import rsim2.scene.JointType;
import rsim2.scene.MotorController;
import rsim2.scene.SceneNode;

import java.util.List;

public class InspectorPanel {
    private final SelectionManager selectionManager;
    private List<Joint> joints;
    private final ImFloat valX = new ImFloat();
    private final ImFloat valY = new ImFloat();
    private final ImFloat valZ = new ImFloat();
    private final ImFloat valScale = new ImFloat();

    public InspectorPanel(SelectionManager selectionManager) {
        this(selectionManager, null);
    }

    public InspectorPanel(SelectionManager selectionManager, List<Joint> joints) {
        this.selectionManager = selectionManager;
        this.joints = joints;
    }

    public void setJoints(List<Joint> joints) {
        this.joints = joints;
    }

    public void render() {
        float displayWidth = ImGui.getIO().getDisplaySizeX();
        float displayHeight = ImGui.getIO().getDisplaySizeY();
        float panelWidth = 300.0f;
        float topOffsetY = 40.0f;

        ImGui.setNextWindowPos(displayWidth - panelWidth, topOffsetY, ImGuiCond.Always);
        ImGui.setNextWindowSize(panelWidth, displayHeight - topOffsetY, ImGuiCond.Always);

        int flags = ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoCollapse;

        SceneNode targetNode = selectionManager != null ? selectionManager.getSelected() : null;

        String windowTitle = targetNode != null ? "Inspector (" + targetNode.getId() + ")" : "Inspector";
        ImGui.begin(windowTitle, flags);

        if (targetNode == null) {
            ImGui.textDisabled("No object selected.");
            ImGui.end();
            return;
        }

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

        ImGui.spacing();

        Vector3f scale = targetNode.getLocalScale();
        valScale.set(scale.x);

        if (ImGui.collapsingHeader("Scale", ImGuiTreeNodeFlags.DefaultOpen)) {
            ImGui.text("Scale");
            ImGui.sameLine(50.0f);
            ImGui.setNextItemWidth(-1.0f);
            if (ImGui.inputFloat("##scale", valScale, 0.05f, 0.5f, "%.3f")) {
                float newScale = Math.max(0.001f, valScale.get());
                targetNode.getLocalScale().set(newScale, newScale, newScale);
            }

            ImGui.spacing();
            if (ImGui.button("Reset Scale", -1.0f, 28.0f)) {
                targetNode.getLocalScale().set(1.0f, 1.0f, 1.0f);
            }
        }

        Joint connectedJoint = null;
        if (joints != null) {
            for (Joint j : joints) {
                if (j.getChildNode() == targetNode) {
                    connectedJoint = j;
                    break;
                }
            }
        }

        if (connectedJoint != null && connectedJoint.getType() == JointType.REVOLUTE && connectedJoint.getMotor() != null) {
            ImGui.spacing();
            if (ImGui.collapsingHeader("Motor Control (" + connectedJoint.getId() + ")", ImGuiTreeNodeFlags.DefaultOpen)) {
                MotorController motor = connectedJoint.getMotor();
                float minDeg = (float) Math.toDegrees(motor.getMinLimitRadians());
                float maxDeg = (float) Math.toDegrees(motor.getMaxLimitRadians());

                float[] angleArr = new float[]{ motor.getTargetAngleRadians() };
                ImGui.text("Target Angle");
                ImGui.setNextItemWidth(-1.0f);
                if (ImGui.sliderAngle("##targetAngle", angleArr, minDeg, maxDeg)) {
                    motor.setTargetAngleRadians(angleArr[0]);
                }

                float currentDeg = (float) Math.toDegrees(connectedJoint.getCurrentAngleRadians());
                float targetDeg = (float) Math.toDegrees(motor.getTargetAngleRadians());

                ImGui.spacing();
                ImGui.text(String.format("Current Angle: %.1f°", currentDeg));
                ImGui.textDisabled(String.format("Target: %.1f° | Max Speed: %.1f rad/s", targetDeg, motor.getMaxSpeedRadiansPerSecond()));

                ImGui.spacing();
                if (ImGui.button("Reset Angle", -1.0f, 26.0f)) {
                    motor.setTargetAngleRadians(0.0f);
                }
            }
        }

        ImGui.spacing();
        ImGui.separator();
        ImGui.spacing();

        if (targetNode.getParent() != null) {
            ImGui.pushStyleColor(ImGuiCol.Button, 0.85f, 0.25f, 0.25f, 1.0f);
            ImGui.pushStyleColor(ImGuiCol.ButtonHovered, 0.95f, 0.35f, 0.35f, 1.0f);
            ImGui.pushStyleColor(ImGuiCol.ButtonActive, 0.70f, 0.15f, 0.15f, 1.0f);
            ImGui.pushStyleColor(ImGuiCol.Text, 1.0f, 1.0f, 1.0f, 1.0f);

            if (ImGui.button("Delete Object", -1.0f, 32.0f)) {
                SceneNode parent = targetNode.getParent();
                if (parent != null) {
                    parent.removeChild(targetNode);
                    selectionManager.clearSelection();
                    targetNode.cleanup();
                }
            }

            ImGui.popStyleColor(4);
        }

        ImGui.end();
    }
}
