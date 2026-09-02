package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiTreeNodeFlags;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
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

    private final ImInt jointPinVal = new ImInt();
    private final ImBoolean jointInvertVal = new ImBoolean();
    private final float[] jointOffsetArr = new float[1];

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
                    ImGui.textColored(0.3f, 0.85f, 1.0f, 1.0f, "Motor Dynamics & Control");

                    MotorController motor = existingJoint.getMotor();
                    float minDeg = (float) Math.toDegrees(motor.getMinLimitRadians());
                    float maxDeg = (float) Math.toDegrees(motor.getMaxLimitRadians());

                    float[] angleArr = new float[] { motor.getTargetAngleRadians() };
                    ImGui.text("Target Angle");
                    ImGui.setNextItemWidth(-1.0f);
                    if (ImGui.sliderAngle("##targetAngle", angleArr, minDeg, maxDeg)) {
                        motor.setTargetAngleRadians(angleArr[0]);
                    }

                    float[] speedPct = new float[] { motor.getTargetSpeedRatio() * 100.0f };
                    ImGui.text("Cruise Speed Ratio");
                    ImGui.setNextItemWidth(-1.0f);
                    if (ImGui.sliderFloat("##speedRatio", speedPct, 5.0f, 100.0f, "%.0f %%")) {
                        motor.setTargetSpeedRatio(speedPct[0] / 100.0f);
                    }
                    if (ImGui.isItemHovered()) {
                        ImGui.setTooltip("Active cruising speed as a percentage of physical top speed");
                    }

                    float[] accelArr = new float[] { motor.getAcceleration() };
                    ImGui.text("Acceleration (Ramp)");
                    ImGui.setNextItemWidth(-1.0f);
                    if (ImGui.sliderFloat("##accelRamp", accelArr, 0.5f, 40.0f, "%.1f rad/s²")) {
                        motor.setAcceleration(accelArr[0]);
                    }
                    if (ImGui.isItemHovered()) {
                        ImGui.setTooltip("Rate of speed increase and deceleration braking ramp");
                    }

                    float currentDeg = (float) Math.toDegrees(existingJoint.getCurrentAngleRadians());
                    float targetDeg = (float) Math.toDegrees(motor.getTargetAngleRadians());
                    float liveVel = motor.getCurrentVelocity();
                    float liveVelDeg = (float) Math.toDegrees(liveVel);
                    String status = motor.getMotionStatus();

                    ImGui.spacing();
                    ImGui.text(String.format("Angle: %.1f deg  ->  Target: %.1f deg", currentDeg, targetDeg));
                    ImGui.text(String.format("Live Velocity: %.2f rad/s (%.1f deg/s)", liveVel, liveVelDeg));

                    ImGui.text("Motion State: ");
                    ImGui.sameLine();
                    if ("ACCELERATING".equals(status)) {
                        ImGui.textColored(0.2f, 0.8f, 1.0f, 1.0f, "ACCELERATING");
                    } else if ("CRUISING".equals(status)) {
                        ImGui.textColored(0.2f, 0.95f, 0.3f, 1.0f, "CRUISING");
                    } else if ("BRAKING".equals(status)) {
                        ImGui.textColored(1.0f, 0.75f, 0.2f, 1.0f, "BRAKING");
                    } else {
                        ImGui.textColored(0.6f, 0.6f, 0.6f, 1.0f, "IDLE");
                    }

                    ImGui.textDisabled(String.format("Max Cap: %.1f rad/s | Limits: [%.0f deg, %.0f deg]",
                            motor.getMaxSpeedRadiansPerSecond(), minDeg, maxDeg));

                    ImGui.spacing();
                    if (ImGui.button("Reset Angle (0 deg)", -1.0f, 24.0f)) {
                        motor.setTargetAngleRadians(0.0f);
                    }
                }

                ImGui.spacing();
                ImGui.separator();
                ImGui.spacing();
                ImGui.textColored(0.2f, 0.7f, 1.0f, 1.0f, "Hardware / Pin Mapping");

                int currentPin = existingJoint.getPin();
                jointPinVal.set(currentPin >= 0 ? currentPin : 9);
                ImGui.setNextItemWidth(140.0f);
                if (ImGui.inputInt("Pin / GPIO / I2C Ch##hwPin", jointPinVal)) {
                    existingJoint.setPin(jointPinVal.get());
                }
                if (ImGui.isItemHovered()) {
                    ImGui.setTooltip("Arduino/ESP32 pin number or Raspberry Pi GPIO / PCA9685 I2C channel (0-15)");
                }

                jointInvertVal.set(existingJoint.isInverted());
                if (ImGui.checkbox("Invert Direction (-1x)##hwInv", jointInvertVal)) {
                    existingJoint.setInverted(jointInvertVal.get());
                }

                jointOffsetArr[0] = existingJoint.getZeroOffsetDeg();
                ImGui.setNextItemWidth(-1.0f);
                if (ImGui.sliderFloat("##hwOffset", jointOffsetArr, -90.0f, 90.0f, "Offset: %.1f deg")) {
                    existingJoint.setZeroOffsetDeg(jointOffsetArr[0]);
                }
                if (ImGui.isItemHovered()) {
                    ImGui.setTooltip("Zero position angular offset in degrees");
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
