package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiTreeNodeFlags;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImFloat;
import org.joml.Vector3f;
import rsim2.core.Engine;
import rsim2.editor.SelectionManager;
import rsim2.editor.commands.CommandHistory;
import rsim2.scene.Joint;
import rsim2.scene.JointType;
import rsim2.scene.MotorController;
import rsim2.scene.SceneNode;

import java.util.ArrayList;
import java.util.List;

public class InspectorPanel {
    private final Engine engine;
    private SceneNode rootNode;
    private final SelectionManager selectionManager;
    private List<Joint> joints;
    private CommandHistory commandHistory;

    private final ImFloat valX = new ImFloat();
    private final ImFloat valY = new ImFloat();
    private final ImFloat valZ = new ImFloat();
    private final ImFloat rotX = new ImFloat();
    private final ImFloat rotY = new ImFloat();
    private final ImFloat rotZ = new ImFloat();
    private final ImFloat valScale = new ImFloat();

    public InspectorPanel(Engine engine, SceneNode rootNode, SelectionManager selectionManager, List<Joint> joints,
            CommandHistory commandHistory) {
        this.engine = engine;
        this.rootNode = rootNode;
        this.selectionManager = selectionManager;
        this.joints = joints != null ? joints : new ArrayList<>();
        this.commandHistory = commandHistory;
    }

    public void setRootNode(SceneNode rootNode) {
        this.rootNode = rootNode;
    }

    public void setJoints(List<Joint> joints) {
        this.joints = joints != null ? joints : new ArrayList<>();
    }

    public void setCommandHistory(CommandHistory commandHistory) {
        this.commandHistory = commandHistory;
    }

    public void render() {
        float displayWidth = ImGui.getIO().getDisplaySizeX();
        float displayHeight = ImGui.getIO().getDisplaySizeY();
        float panelWidth = 320.0f;
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
            boolean posChanged = false;

            ImGui.text("X");
            ImGui.sameLine(30.0f);
            ImGui.setNextItemWidth(-1.0f);
            if (ImGui.inputFloat("##posX", valX, 0.1f, 1.0f, "%.3f"))
                posChanged = true;

            ImGui.text("Y");
            ImGui.sameLine(30.0f);
            ImGui.setNextItemWidth(-1.0f);
            if (ImGui.inputFloat("##posY", valY, 0.1f, 1.0f, "%.3f"))
                posChanged = true;

            ImGui.text("Z");
            ImGui.sameLine(30.0f);
            ImGui.setNextItemWidth(-1.0f);
            if (ImGui.inputFloat("##posZ", valZ, 0.1f, 1.0f, "%.3f"))
                posChanged = true;

            if (posChanged) {
                targetNode.getLocalPosition().set(valX.get(), valY.get(), valZ.get());
            }

            ImGui.spacing();
            if (ImGui.button("Reset Position", -1.0f, 26.0f)) {
                targetNode.getLocalPosition().set(0.0f, 0.0f, 0.0f);
            }
        }

        ImGui.spacing();

        Vector3f euler = new Vector3f();
        targetNode.getLocalRotation().getEulerAnglesXYZ(euler);
        rotX.set((float) Math.toDegrees(euler.x));
        rotY.set((float) Math.toDegrees(euler.y));
        rotZ.set((float) Math.toDegrees(euler.z));

        if (ImGui.collapsingHeader("Rotation", ImGuiTreeNodeFlags.DefaultOpen)) {
            boolean rotChanged = false;

            ImGui.text("X");
            ImGui.sameLine(30.0f);
            ImGui.setNextItemWidth(-1.0f);
            if (ImGui.inputFloat("##rotX", rotX, 1.0f, 15.0f, "%.1f deg"))
                rotChanged = true;

            ImGui.text("Y");
            ImGui.sameLine(30.0f);
            ImGui.setNextItemWidth(-1.0f);
            if (ImGui.inputFloat("##rotY", rotY, 1.0f, 15.0f, "%.1f deg"))
                rotChanged = true;

            ImGui.text("Z");
            ImGui.sameLine(30.0f);
            ImGui.setNextItemWidth(-1.0f);
            if (ImGui.inputFloat("##rotZ", rotZ, 1.0f, 15.0f, "%.1f deg"))
                rotChanged = true;

            if (rotChanged) {
                targetNode.getLocalRotation().rotationXYZ(
                        (float) Math.toRadians(rotX.get()),
                        (float) Math.toRadians(rotY.get()),
                        (float) Math.toRadians(rotZ.get()));
            }

            ImGui.spacing();
            if (ImGui.button("+90 X", 90.0f, 24.0f)) {
                targetNode.getLocalRotation().rotateX((float) Math.toRadians(90));
            }
            ImGui.sameLine();
            if (ImGui.button("+90 Y", 90.0f, 24.0f)) {
                targetNode.getLocalRotation().rotateY((float) Math.toRadians(90));
            }
            ImGui.sameLine();
            if (ImGui.button("+90 Z", 90.0f, 24.0f)) {
                targetNode.getLocalRotation().rotateZ((float) Math.toRadians(90));
            }

            ImGui.spacing();
            if (ImGui.button("Reset Rotation", -1.0f, 26.0f)) {
                targetNode.getLocalRotation().identity();
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
            if (ImGui.button("Reset Scale", -1.0f, 26.0f)) {
                targetNode.getLocalScale().set(1.0f, 1.0f, 1.0f);
            }
        }

        ImGui.spacing();

        Joint existingJoint = findJointForChild(targetNode);

        if (existingJoint != null) {
            if (ImGui.collapsingHeader("Joint: " + existingJoint.getId(), ImGuiTreeNodeFlags.DefaultOpen)) {
                ImGui.text("Parent:");
                ImGui.sameLine(70.0f);
                ImGui.textColored(0.3f, 0.8f, 1.0f, 1.0f,
                        existingJoint.getParentNode() != null ? existingJoint.getParentNode().getId() : "None");

                ImGui.text("Type:");
                ImGui.sameLine(70.0f);
                ImGui.text(existingJoint.getType().name());

                Vector3f axis = existingJoint.getAxis();
                ImGui.text("Axis:");
                ImGui.sameLine(70.0f);
                ImGui.text(String.format("(%.1f, %.1f, %.1f)", axis.x, axis.y, axis.z));

                if (existingJoint.getType() == JointType.REVOLUTE && existingJoint.getMotor() != null) {
                    ImGui.spacing();
                    ImGui.separator();
                    ImGui.spacing();
                    ImGui.text("Motor Control");

                    MotorController motor = existingJoint.getMotor();
                    float minDeg = (float) Math.toDegrees(motor.getMinLimitRadians());
                    float maxDeg = (float) Math.toDegrees(motor.getMaxLimitRadians());

                    float[] angleArr = new float[] { motor.getTargetAngleRadians() };
                    ImGui.text("Target Angle");
                    ImGui.setNextItemWidth(-1.0f);
                    if (ImGui.sliderAngle("##targetAngle", angleArr, minDeg, maxDeg)) {
                        motor.setTargetAngleRadians(angleArr[0]);
                    }

                    float currentDeg = (float) Math.toDegrees(existingJoint.getCurrentAngleRadians());
                    float targetDeg = (float) Math.toDegrees(motor.getTargetAngleRadians());

                    ImGui.spacing();
                    ImGui.text(String.format("Current: %.1f deg | Target: %.1f deg", currentDeg, targetDeg));
                    ImGui.textDisabled(String.format("Speed: %.1f rad/s | Limits: [%.0f deg, %.0f deg]",
                            motor.getMaxSpeedRadiansPerSecond(), minDeg, maxDeg));

                    ImGui.spacing();
                    if (ImGui.button("Reset Angle (0 deg)", -1.0f, 26.0f)) {
                        motor.setTargetAngleRadians(0.0f);
                    }
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
                if (engine != null) {
                    engine.deleteNode(targetNode);
                } else {
                    SceneNode parent = targetNode.getParent();
                    if (parent != null) {
                        parent.removeChild(targetNode);
                        selectionManager.clearSelection();
                        targetNode.cleanup();
                    }
                }
            }

            ImGui.popStyleColor(4);
        }

        ImGui.end();
    }

    private Joint findJointForChild(SceneNode childNode) {
        if (joints == null || childNode == null)
            return null;
        for (Joint j : joints) {
            if (j.getChildNode() == childNode) {
                return j;
            }
        }
        return null;
    }
}
