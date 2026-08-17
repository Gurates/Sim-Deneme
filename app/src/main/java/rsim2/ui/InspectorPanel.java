package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiTreeNodeFlags;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.joml.Matrix4f;
import org.joml.Quaternionf;
import org.joml.Vector3f;
import rsim2.core.Engine;
import rsim2.editor.AlignmentHelper;
import rsim2.editor.PointAlignTool;
import rsim2.editor.SelectionManager;
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
    private PointAlignTool pointAlignTool;

    private final ImFloat valX = new ImFloat();
    private final ImFloat valY = new ImFloat();
    private final ImFloat valZ = new ImFloat();
    private final ImFloat rotX = new ImFloat();
    private final ImFloat rotY = new ImFloat();
    private final ImFloat rotZ = new ImFloat();
    private final ImFloat valScale = new ImFloat();

    private SceneNode lastSelectedNode = null;
    private SceneNode pendingParentNode = null;
    private final ImInt selectedTypeIdx = new ImInt(0);
    private final Vector3f selectedAxis = new Vector3f(0.0f, 1.0f, 0.0f);
    private final ImFloat originX = new ImFloat();
    private final ImFloat originY = new ImFloat();
    private final ImFloat originZ = new ImFloat();
    private String jointStatusMessage = null;
    private boolean jointErrorStatus = false;

    public InspectorPanel(Engine engine, SceneNode rootNode, SelectionManager selectionManager, List<Joint> joints, PointAlignTool pointAlignTool) {
        this.engine = engine;
        this.rootNode = rootNode;
        this.selectionManager = selectionManager;
        this.joints = joints != null ? joints : new ArrayList<>();
        this.pointAlignTool = pointAlignTool;
    }

    public void setRootNode(SceneNode rootNode) {
        this.rootNode = rootNode;
    }

    public void setJoints(List<Joint> joints) {
        this.joints = joints != null ? joints : new ArrayList<>();
    }

    public void setPointAlignTool(PointAlignTool pointAlignTool) {
        this.pointAlignTool = pointAlignTool;
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

        if (targetNode != lastSelectedNode) {
            lastSelectedNode = targetNode;
            jointStatusMessage = null;
            jointErrorStatus = false;
            if (pointAlignTool != null) {
                pointAlignTool.clearPoints();
            }

            Joint existingJoint = findJointForChild(targetNode);
            if (existingJoint != null) {
                pendingParentNode = existingJoint.getParentNode();
                selectedTypeIdx.set(existingJoint.getType() == JointType.REVOLUTE ? 0 : 1);
                selectedAxis.set(existingJoint.getAxis());
            } else if (targetNode.getParent() != null && !targetNode.getParent().getId().equalsIgnoreCase("world")) {
                pendingParentNode = targetNode.getParent();
                selectedTypeIdx.set(0);
                selectedAxis.set(0.0f, 1.0f, 0.0f);
            } else {
                pendingParentNode = null;
                selectedTypeIdx.set(0);
                selectedAxis.set(0.0f, 1.0f, 0.0f);
            }

            originX.set(targetNode.getLocalPosition().x);
            originY.set(targetNode.getLocalPosition().y);
            originZ.set(targetNode.getLocalPosition().z);
        }

        Vector3f pos = targetNode.getLocalPosition();
        valX.set(pos.x);
        valY.set(pos.y);
        valZ.set(pos.z);

        if (ImGui.collapsingHeader("Position", ImGuiTreeNodeFlags.DefaultOpen)) {
            boolean posChanged = false;

            ImGui.text("X"); ImGui.sameLine(30.0f); ImGui.setNextItemWidth(-1.0f);
            if (ImGui.inputFloat("##posX", valX, 0.1f, 1.0f, "%.2f")) posChanged = true;

            ImGui.text("Y"); ImGui.sameLine(30.0f); ImGui.setNextItemWidth(-1.0f);
            if (ImGui.inputFloat("##posY", valY, 0.1f, 1.0f, "%.2f")) posChanged = true;

            ImGui.text("Z"); ImGui.sameLine(30.0f); ImGui.setNextItemWidth(-1.0f);
            if (ImGui.inputFloat("##posZ", valZ, 0.1f, 1.0f, "%.2f")) posChanged = true;

            if (posChanged) {
                targetNode.getLocalPosition().set(valX.get(), valY.get(), valZ.get());
                originX.set(valX.get());
                originY.set(valY.get());
                originZ.set(valZ.get());
            }

            ImGui.spacing();
            if (ImGui.button("Reset Position", -1.0f, 26.0f)) {
                targetNode.getLocalPosition().set(0.0f, 0.5f, 0.0f);
                originX.set(0.0f);
                originY.set(0.5f);
                originZ.set(0.0f);
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

            ImGui.text("X"); ImGui.sameLine(30.0f); ImGui.setNextItemWidth(-1.0f);
            if (ImGui.inputFloat("##rotX", rotX, 1.0f, 15.0f, "%.1f deg")) rotChanged = true;

            ImGui.text("Y"); ImGui.sameLine(30.0f); ImGui.setNextItemWidth(-1.0f);
            if (ImGui.inputFloat("##rotY", rotY, 1.0f, 15.0f, "%.1f deg")) rotChanged = true;

            ImGui.text("Z"); ImGui.sameLine(30.0f); ImGui.setNextItemWidth(-1.0f);
            if (ImGui.inputFloat("##rotZ", rotZ, 1.0f, 15.0f, "%.1f deg")) rotChanged = true;

            if (rotChanged) {
                targetNode.getLocalRotation().rotationXYZ(
                        (float) Math.toRadians(rotX.get()),
                        (float) Math.toRadians(rotY.get()),
                        (float) Math.toRadians(rotZ.get())
                );
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

        if (ImGui.collapsingHeader("Joint / Connected Body", ImGuiTreeNodeFlags.DefaultOpen)) {
            ImGui.text("Connected Body (Parent):");

            String parentLabel = (pendingParentNode != null)
                    ? pendingParentNode.getId()
                    : "None (Drag a node from Hierarchy)";

            if (pendingParentNode == null) {
                ImGui.pushStyleColor(ImGuiCol.Text, 0.6f, 0.6f, 0.6f, 1.0f);
            }
            ImGui.button(parentLabel + "##connBodyBtn", -1.0f, 26.0f);
            if (pendingParentNode == null) {
                ImGui.popStyleColor();
            }

            if (ImGui.beginDragDropTarget()) {
                Object payload = ImGui.acceptDragDropPayload("SCENE_NODE");
                if (payload != null) {
                    SceneNode dropped = HierarchyPanel.draggedNode;
                    if (dropped == null && payload instanceof String) {
                        dropped = findNodeById(rootNode, (String) payload);
                    }
                    if (dropped != null) {
                        if (dropped == targetNode) {
                            jointStatusMessage = "Cannot connect a node to itself!";
                            jointErrorStatus = true;
                        } else if (isAncestor(targetNode, dropped)) {
                            jointStatusMessage = "Invalid parent: would create a cycle in tree!";
                            jointErrorStatus = true;
                        } else {
                            pendingParentNode = dropped;
                            jointStatusMessage = null;
                            jointErrorStatus = false;
                            originX.set(targetNode.getLocalPosition().x);
                            originY.set(targetNode.getLocalPosition().y);
                            originZ.set(targetNode.getLocalPosition().z);
                        }
                    }
                }
                ImGui.endDragDropTarget();
            }

            if (pendingParentNode != null) {
                ImGui.spacing();

                ImGui.text("Joint Type");
                boolean isRev = (selectedTypeIdx.get() == 0);
                if (isRev) ImGui.pushStyleColor(ImGuiCol.Button, 0.2f, 0.6f, 0.9f, 1.0f);
                if (ImGui.button("Revolute", 130.0f, 24.0f)) {
                    selectedTypeIdx.set(0);
                }
                if (isRev) ImGui.popStyleColor();

                ImGui.sameLine();
                boolean isFixed = (selectedTypeIdx.get() == 1);
                if (isFixed) ImGui.pushStyleColor(ImGuiCol.Button, 0.2f, 0.6f, 0.9f, 1.0f);
                if (ImGui.button("Fixed", 130.0f, 24.0f)) {
                    selectedTypeIdx.set(1);
                }
                if (isFixed) ImGui.popStyleColor();

                if (selectedTypeIdx.get() == 0) {
                    ImGui.spacing();
                    ImGui.text("Rotation Axis");

                    boolean isX = selectedAxis.x == 1.0f;
                    if (isX) ImGui.pushStyleColor(ImGuiCol.Button, 0.2f, 0.6f, 0.9f, 1.0f);
                    if (ImGui.button("X##axisX", 85.0f, 24.0f)) {
                        selectedAxis.set(1.0f, 0.0f, 0.0f);
                    }
                    if (isX) ImGui.popStyleColor();

                    ImGui.sameLine();
                    boolean isY = selectedAxis.y == 1.0f;
                    if (isY) ImGui.pushStyleColor(ImGuiCol.Button, 0.2f, 0.6f, 0.9f, 1.0f);
                    if (ImGui.button("Y##axisY", 85.0f, 24.0f)) {
                        selectedAxis.set(0.0f, 1.0f, 0.0f);
                    }
                    if (isY) ImGui.popStyleColor();

                    ImGui.sameLine();
                    boolean isZ = selectedAxis.z == 1.0f;
                    if (isZ) ImGui.pushStyleColor(ImGuiCol.Button, 0.2f, 0.6f, 0.9f, 1.0f);
                    if (ImGui.button("Z##axisZ", 85.0f, 24.0f)) {
                        selectedAxis.set(0.0f, 0.0f, 1.0f);
                    }
                    if (isZ) ImGui.popStyleColor();
                }

                ImGui.spacing();
                ImGui.text("Origin Position (Live Preview)");

                ImGui.text("X"); ImGui.sameLine(25.0f); ImGui.setNextItemWidth(-1.0f);
                boolean xChg = ImGui.inputFloat("##origX", originX, 0.05f, 0.5f, "%.2f");

                ImGui.text("Y"); ImGui.sameLine(25.0f); ImGui.setNextItemWidth(-1.0f);
                boolean yChg = ImGui.inputFloat("##origY", originY, 0.05f, 0.5f, "%.2f");

                ImGui.text("Z"); ImGui.sameLine(25.0f); ImGui.setNextItemWidth(-1.0f);
                boolean zChg = ImGui.inputFloat("##origZ", originZ, 0.05f, 0.5f, "%.2f");

                if (xChg || yChg || zChg) {
                    targetNode.getLocalPosition().set(originX.get(), originY.get(), originZ.get());
                } else if (!ImGui.isAnyItemActive()) {
                    originX.set(targetNode.getLocalPosition().x);
                    originY.set(targetNode.getLocalPosition().y);
                    originZ.set(targetNode.getLocalPosition().z);
                }

                ImGui.spacing();

                if (ImGui.button("Snap to Top", 130.0f, 24.0f)) {
                    AlignmentHelper.snapToTop(pendingParentNode, targetNode);
                    originY.set(targetNode.getLocalPosition().y);
                }
                ImGui.sameLine();
                if (ImGui.button("Center X/Z", 130.0f, 24.0f)) {
                    AlignmentHelper.snapCenterXZ(pendingParentNode, targetNode);
                    originX.set(targetNode.getLocalPosition().x);
                    originZ.set(targetNode.getLocalPosition().z);
                }

                if (ImGui.button("Reset to Origin (0,0,0)", -1.0f, 22.0f)) {
                    targetNode.getLocalPosition().set(0.0f, 0.0f, 0.0f);
                    originX.set(0.0f);
                    originY.set(0.0f);
                    originZ.set(0.0f);
                }

                if (pointAlignTool != null) {
                    pointAlignTool.setNodes(pendingParentNode, targetNode);

                    ImGui.spacing();
                    ImGui.separator();
                    ImGui.spacing();

                    ImGui.text("Point Align (Pick Surface Points)");

                    boolean pickP = pointAlignTool.isPickingParent();
                    if (pickP) ImGui.pushStyleColor(ImGuiCol.Button, 0.9f, 0.5f, 0.1f, 1.0f);
                    String pBtnText = pickP ? "[Click Parent in 3D]" : "Pick Pt on Parent";
                    if (ImGui.button(pBtnText, 130.0f, 24.0f)) {
                        if (pickP) pointAlignTool.cancelPicking();
                        else pointAlignTool.startPickParentPoint();
                    }
                    if (pickP) ImGui.popStyleColor();

                    ImGui.sameLine();
                    boolean pickC = pointAlignTool.isPickingChild();
                    if (pickC) ImGui.pushStyleColor(ImGuiCol.Button, 0.75f, 0.25f, 0.85f, 1.0f);
                    String cBtnText = pickC ? "[Click Child in 3D]" : "Pick Pt on Child";
                    if (ImGui.button(cBtnText, 130.0f, 24.0f)) {
                        if (pickC) pointAlignTool.cancelPicking();
                        else pointAlignTool.startPickChildPoint();
                    }
                    if (pickC) ImGui.popStyleColor();

                    if (pointAlignTool.getPickedParentPointWorld() != null) {
                        Vector3f pt = pointAlignTool.getPickedParentPointWorld();
                        ImGui.textColored(1.0f, 0.6f, 0.1f, 1.0f, String.format("Parent: (%.2f, %.2f, %.2f)", pt.x, pt.y, pt.z));
                    }
                    if (pointAlignTool.getPickedChildPointWorld() != null) {
                        Vector3f pt = pointAlignTool.getPickedChildPointWorld();
                        ImGui.textColored(0.85f, 0.35f, 0.95f, 1.0f, String.format("Child:  (%.2f, %.2f, %.2f)", pt.x, pt.y, pt.z));
                    }

                    boolean canAlign = pointAlignTool.canAlign();
                    ImGui.beginDisabled(!canAlign);
                    if (ImGui.button("Align Points", 130.0f, 24.0f)) {
                        pointAlignTool.align();
                        originX.set(targetNode.getLocalPosition().x);
                        originY.set(targetNode.getLocalPosition().y);
                        originZ.set(targetNode.getLocalPosition().z);
                    }
                    ImGui.endDisabled();

                    ImGui.sameLine();
                    if (ImGui.button("Clear Points", 130.0f, 24.0f)) {
                        pointAlignTool.clearPoints();
                    }
                }

                ImGui.spacing();
                ImGui.separator();
                ImGui.spacing();

                String applyBtnText = (existingJoint != null) ? "Update Joint" : "Apply Joint (Connect)";
                ImGui.pushStyleColor(ImGuiCol.Button, 0.2f, 0.65f, 0.3f, 1.0f);
                ImGui.pushStyleColor(ImGuiCol.ButtonHovered, 0.25f, 0.75f, 0.35f, 1.0f);
                if (ImGui.button(applyBtnText, -1.0f, 28.0f)) {
                    applyJoint(pendingParentNode, targetNode);
                }
                ImGui.popStyleColor(2);

                if (existingJoint != null) {
                    ImGui.spacing();
                    ImGui.pushStyleColor(ImGuiCol.Button, 0.7f, 0.25f, 0.25f, 1.0f);
                    if (ImGui.button("Remove Joint (Disconnect)", -1.0f, 24.0f)) {
                        removeJoint(existingJoint);
                    }
                    ImGui.popStyleColor();
                }
            } else {
                ImGui.spacing();
                ImGui.textDisabled("Drag any body from Hierarchy into the box above to create a joint.");
            }

            if (jointStatusMessage != null) {
                ImGui.spacing();
                if (jointErrorStatus) {
                    ImGui.textColored(1.0f, 0.3f, 0.3f, 1.0f, jointStatusMessage);
                } else {
                    ImGui.textColored(0.3f, 0.9f, 0.3f, 1.0f, jointStatusMessage);
                }
            }
        }

        if (existingJoint != null && existingJoint.getType() == JointType.REVOLUTE && existingJoint.getMotor() != null) {
            ImGui.spacing();
            if (ImGui.collapsingHeader("Motor Control (" + existingJoint.getId() + ")", ImGuiTreeNodeFlags.DefaultOpen)) {
                MotorController motor = existingJoint.getMotor();
                float minDeg = (float) Math.toDegrees(motor.getMinLimitRadians());
                float maxDeg = (float) Math.toDegrees(motor.getMaxLimitRadians());

                float[] angleArr = new float[]{ motor.getTargetAngleRadians() };
                ImGui.text("Target Angle");
                ImGui.setNextItemWidth(-1.0f);
                if (ImGui.sliderAngle("##targetAngle", angleArr, minDeg, maxDeg)) {
                    motor.setTargetAngleRadians(angleArr[0]);
                }

                float currentDeg = (float) Math.toDegrees(existingJoint.getCurrentAngleRadians());
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

    private void applyJoint(SceneNode parentNode, SceneNode childNode) {
        if (parentNode == null || childNode == null) return;

        if (childNode.getParent() != null) {
            childNode.getParent().removeChild(childNode);
        }
        parentNode.addChild(childNode);
        childNode.getLocalPosition().set(originX.get(), originY.get(), originZ.get());

        joints.removeIf(j -> j.getChildNode() == childNode);

        String baseJointId = parentNode.getId() + "_" + childNode.getId() + "_joint";
        String uniqueJointId = baseJointId;
        int counter = 1;
        while (jointExists(uniqueJointId, joints)) {
            uniqueJointId = baseJointId + "_" + counter++;
        }

        JointType type = (selectedTypeIdx.get() == 0) ? JointType.REVOLUTE : JointType.FIXED;
        Vector3f axis = (type == JointType.REVOLUTE) ? new Vector3f(selectedAxis) : new Vector3f(0.0f, 0.0f, 0.0f);

        Joint newJoint = new Joint(uniqueJointId, parentNode, childNode, type, axis);
        joints.add(newJoint);

        if (engine != null) {
            engine.autoSaveProject();
        }

        jointStatusMessage = "Joint applied: " + uniqueJointId;
        jointErrorStatus = false;
    }

    private void removeJoint(Joint joint) {
        if (joint == null) return;

        SceneNode childNode = joint.getChildNode();
        SceneNode parentNode = joint.getParentNode();

        if (childNode != null) {
            Matrix4f worldTransform = childNode.getWorldTransform();
            Vector3f worldPos = new Vector3f();
            worldTransform.getTranslation(worldPos);

            Quaternionf worldRot = new Quaternionf();
            worldTransform.getNormalizedRotation(worldRot);

            Vector3f worldScale = new Vector3f();
            worldTransform.getScale(worldScale);

            if (childNode.getParent() != null && childNode.getParent() == parentNode) {
                childNode.getParent().removeChild(childNode);
                if (rootNode != null) {
                    rootNode.addChild(childNode);
                }
                childNode.getLocalPosition().set(worldPos);
                childNode.getLocalRotation().set(worldRot);
                childNode.getLocalScale().set(worldScale);
            }
        }

        joints.remove(joint);
        pendingParentNode = null;

        if (engine != null) {
            engine.autoSaveProject();
        }

        jointStatusMessage = "Joint removed.";
        jointErrorStatus = false;
    }

    private Joint findJointForChild(SceneNode childNode) {
        if (joints == null || childNode == null) return null;
        for (Joint j : joints) {
            if (j.getChildNode() == childNode) {
                return j;
            }
        }
        return null;
    }

    private boolean jointExists(String id, List<Joint> jointList) {
        if (jointList == null) return false;
        for (Joint j : jointList) {
            if (j.getId().equals(id)) return true;
        }
        return false;
    }

    private boolean isAncestor(SceneNode potentialAncestor, SceneNode node) {
        if (potentialAncestor == null || node == null) return false;
        SceneNode current = node.getParent();
        while (current != null) {
            if (current == potentialAncestor) {
                return true;
            }
            current = current.getParent();
        }
        return false;
    }

    private SceneNode findNodeById(SceneNode root, String id) {
        if (root == null || id == null) return null;
        if (root.getId().equals(id)) return root;
        for (SceneNode child : root.getChildren()) {
            SceneNode found = findNodeById(child, id);
            if (found != null) return found;
        }
        return null;
    }
}
