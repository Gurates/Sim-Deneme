package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.joml.Matrix4f;
import org.joml.Quaternionf;
import org.joml.Vector3f;
import rsim2.core.Engine;
import rsim2.editor.AlignmentHelper;
import rsim2.scene.Joint;
import rsim2.scene.JointType;
import rsim2.scene.SceneNode;

import java.util.ArrayList;
import java.util.List;

public class JointToolPanel {
    private final Engine engine;
    private SceneNode rootNode;
    private List<Joint> allJoints;
    private final Runnable onProjectChanged;

    private final ImInt selectedTypeIdx = new ImInt(0);
    private final ImInt selectedParentIdx = new ImInt(0);
    private final ImInt selectedChildIdx = new ImInt(0);
    private final Vector3f selectedAxis = new Vector3f(0.0f, 1.0f, 0.0f);
    private final ImFloat originX = new ImFloat(0.0f);
    private final ImFloat originY = new ImFloat(0.0f);
    private final ImFloat originZ = new ImFloat(0.0f);

    private SceneNode activePreviewChild = null;
    private Vector3f originalChildPos = null;

    private String statusMessage = null;
    private boolean isErrorStatus = false;

    public JointToolPanel(Engine engine, SceneNode rootNode, List<Joint> allJoints) {
        this(engine, rootNode, allJoints, null);
    }

    public JointToolPanel(Engine engine, SceneNode rootNode, List<Joint> allJoints, Runnable onProjectChanged) {
        this.engine = engine;
        this.rootNode = rootNode;
        this.allJoints = allJoints != null ? allJoints : new ArrayList<>();
        this.onProjectChanged = onProjectChanged;
    }

    public void setRootNode(SceneNode rootNode) {
        revertPreviewPosition();
        this.rootNode = rootNode;
        this.activePreviewChild = null;
        this.originalChildPos = null;
    }

    public void setJoints(List<Joint> allJoints) {
        this.allJoints = allJoints != null ? allJoints : new ArrayList<>();
    }

    public SceneNode getActivePreviewChild() {
        return activePreviewChild;
    }

    public void render() {
        float displayHeight = ImGui.getIO().getDisplaySizeY();
        float panelWidth = 280.0f;
        float topOffsetY = 40.0f;
        float totalHeight = displayHeight - topOffsetY;
        float hierarchyHeight = totalHeight * 0.5f;
        float toolPosY = topOffsetY + hierarchyHeight;
        float toolHeight = totalHeight - hierarchyHeight;

        ImGui.setNextWindowPos(0.0f, toolPosY, ImGuiCond.Always);
        ImGui.setNextWindowSize(panelWidth, toolHeight, ImGuiCond.Always);

        int flags = ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoCollapse;

        ImGui.begin("Joint Tool", flags);

        List<SceneNode> flatNodes = new ArrayList<>();
        flattenNodes(rootNode, flatNodes);

        if (flatNodes.isEmpty()) {
            revertPreviewPosition();
            ImGui.textDisabled("No scene nodes available.");
            ImGui.end();
            return;
        }

        String[] nodeNames = flatNodes.stream().map(SceneNode::getId).toArray(String[]::new);

        if (selectedParentIdx.get() < 0 || selectedParentIdx.get() >= nodeNames.length) {
            selectedParentIdx.set(0);
        }
        if (selectedChildIdx.get() < 0 || selectedChildIdx.get() >= nodeNames.length) {
            selectedChildIdx.set(0);
        }

        ImGui.text("Joint Type");
        boolean isRev = (selectedTypeIdx.get() == 0);
        if (isRev)
            ImGui.pushStyleColor(ImGuiCol.Button, 0.2f, 0.6f, 0.9f, 1.0f);
        if (ImGui.button("Revolute", 115.0f, 24.0f)) {
            selectedTypeIdx.set(0);
        }
        if (isRev)
            ImGui.popStyleColor();

        ImGui.sameLine();
        boolean isFixed = (selectedTypeIdx.get() == 1);
        if (isFixed)
            ImGui.pushStyleColor(ImGuiCol.Button, 0.2f, 0.6f, 0.9f, 1.0f);
        if (ImGui.button("Fixed", 115.0f, 24.0f)) {
            selectedTypeIdx.set(1);
        }
        if (isFixed)
            ImGui.popStyleColor();

        ImGui.spacing();

        ImGui.text("Parent Link");
        ImGui.setNextItemWidth(-1.0f);
        ImGui.combo("##parentCombo", selectedParentIdx, nodeNames);

        ImGui.spacing();

        ImGui.text("Child Link");
        ImGui.setNextItemWidth(-1.0f);
        if (ImGui.combo("##childCombo", selectedChildIdx, nodeNames) || activePreviewChild == null) {
            SceneNode newChild = flatNodes.get(selectedChildIdx.get());
            if (newChild != activePreviewChild) {
                revertPreviewPosition();
                activePreviewChild = newChild;
                if (activePreviewChild != null) {
                    originalChildPos = new Vector3f(activePreviewChild.getLocalPosition());
                    originX.set(activePreviewChild.getLocalPosition().x);
                    originY.set(activePreviewChild.getLocalPosition().y);
                    originZ.set(activePreviewChild.getLocalPosition().z);
                }
            }
        }

        SceneNode currentChild = (selectedChildIdx.get() >= 0 && selectedChildIdx.get() < flatNodes.size())
                ? flatNodes.get(selectedChildIdx.get())
                : null;

        if (selectedTypeIdx.get() == 0) {
            ImGui.spacing();
            ImGui.text("Rotation Axis");

            boolean isX = selectedAxis.x == 1.0f;
            if (isX)
                ImGui.pushStyleColor(ImGuiCol.Button, 0.2f, 0.6f, 0.9f, 1.0f);
            if (ImGui.button("X##axisX", 75.0f, 24.0f)) {
                selectedAxis.set(1.0f, 0.0f, 0.0f);
            }
            if (isX)
                ImGui.popStyleColor();

            ImGui.sameLine();
            boolean isY = selectedAxis.y == 1.0f;
            if (isY)
                ImGui.pushStyleColor(ImGuiCol.Button, 0.2f, 0.6f, 0.9f, 1.0f);
            if (ImGui.button("Y##axisY", 75.0f, 24.0f)) {
                selectedAxis.set(0.0f, 1.0f, 0.0f);
            }
            if (isY)
                ImGui.popStyleColor();

            ImGui.sameLine();
            boolean isZ = selectedAxis.z == 1.0f;
            if (isZ)
                ImGui.pushStyleColor(ImGuiCol.Button, 0.2f, 0.6f, 0.9f, 1.0f);
            if (ImGui.button("Z##axisZ", 75.0f, 24.0f)) {
                selectedAxis.set(0.0f, 0.0f, 1.0f);
            }
            if (isZ)
                ImGui.popStyleColor();
        }

        ImGui.spacing();
        ImGui.text("Origin Position (Live Preview)");

        ImGui.text("X");
        ImGui.sameLine(25.0f);
        ImGui.setNextItemWidth(-1.0f);
        boolean xChg = ImGui.inputFloat("##origX", originX, 0.05f, 0.5f, "%.2f");

        ImGui.text("Y");
        ImGui.sameLine(25.0f);
        ImGui.setNextItemWidth(-1.0f);
        boolean yChg = ImGui.inputFloat("##origY", originY, 0.05f, 0.5f, "%.2f");

        ImGui.text("Z");
        ImGui.sameLine(25.0f);
        ImGui.setNextItemWidth(-1.0f);
        boolean zChg = ImGui.inputFloat("##origZ", originZ, 0.05f, 0.5f, "%.2f");

        if ((xChg || yChg || zChg) && currentChild != null) {
            currentChild.getLocalPosition().set(originX.get(), originY.get(), originZ.get());
        } else if (currentChild != null && !ImGui.isAnyItemActive()) {
            originX.set(currentChild.getLocalPosition().x);
            originY.set(currentChild.getLocalPosition().y);
            originZ.set(currentChild.getLocalPosition().z);
        }

        ImGui.spacing();

        SceneNode pNode = (selectedParentIdx.get() >= 0 && selectedParentIdx.get() < flatNodes.size())
                ? flatNodes.get(selectedParentIdx.get())
                : null;

        if (ImGui.button("Snap to Top", 115.0f, 24.0f)) {
            if (pNode != null && currentChild != null) {
                AlignmentHelper.snapToTop(pNode, currentChild);
                originY.set(currentChild.getLocalPosition().y);
            }
        }
        ImGui.sameLine();
        if (ImGui.button("Center X/Z", 115.0f, 24.0f)) {
            if (pNode != null && currentChild != null) {
                AlignmentHelper.snapCenterXZ(pNode, currentChild);
                originX.set(currentChild.getLocalPosition().x);
                originZ.set(currentChild.getLocalPosition().z);
            }
        }

        if (ImGui.button("Reset to Origin (0,0,0)", -1.0f, 22.0f)) {
            if (currentChild != null) {
                currentChild.getLocalPosition().set(0.0f, 0.0f, 0.0f);
                originX.set(0.0f);
                originY.set(0.0f);
                originZ.set(0.0f);
            }
        }

        ImGui.spacing();

        if (ImGui.button("Create Joint", -1.0f, 28.0f)) {
            SceneNode parentNode = flatNodes.get(selectedParentIdx.get());
            SceneNode childNode = flatNodes.get(selectedChildIdx.get());

            if (parentNode == childNode) {
                statusMessage = "Parent and Child cannot be the same node!";
                isErrorStatus = true;
            } else if (isAncestor(childNode, parentNode)) {
                statusMessage = "Connection creates a cycle in tree!";
                isErrorStatus = true;
            } else {
                if (childNode.getParent() != null) {
                    childNode.getParent().removeChild(childNode);
                }
                parentNode.addChild(childNode);
                childNode.getLocalPosition().set(originX.get(), originY.get(), originZ.get());

                originalChildPos = new Vector3f(childNode.getLocalPosition());

                boolean replacedPrevious = allJoints.removeIf(j -> j.getChildNode() == childNode);

                String baseJointId = parentNode.getId() + "_" + childNode.getId() + "_joint";
                String uniqueJointId = baseJointId;
                int counter = 1;
                while (jointExists(uniqueJointId, allJoints)) {
                    uniqueJointId = baseJointId + "_" + counter++;
                }

                JointType type = (selectedTypeIdx.get() == 0) ? JointType.REVOLUTE : JointType.FIXED;
                Vector3f axis = (type == JointType.REVOLUTE) ? new Vector3f(selectedAxis)
                        : new Vector3f(0.0f, 0.0f, 0.0f);

                Joint newJoint = new Joint(uniqueJointId, parentNode, childNode, type, axis);
                allJoints.add(newJoint);

                if (onProjectChanged != null) {
                    onProjectChanged.run();
                } else if (engine != null) {
                    engine.autoSaveProject();
                }

                String currentPath = engine != null ? engine.getCurrentProjectPath() : null;
                String extraMsg = replacedPrevious ? " (replaced previous joint)" : "";
                if (currentPath != null && !currentPath.trim().isEmpty()) {
                    statusMessage = "Joint created: " + uniqueJointId + extraMsg + " (Saved)";
                } else {
                    statusMessage = "Joint created: " + uniqueJointId + extraMsg;
                }

                isErrorStatus = false;
            }
        }

        if (statusMessage != null) {
            ImGui.spacing();
            if (isErrorStatus) {
                ImGui.textColored(1.0f, 0.3f, 0.3f, 1.0f, statusMessage);
            } else {
                ImGui.textColored(0.3f, 0.9f, 0.3f, 1.0f, statusMessage);
            }
        }

        ImGui.spacing();
        ImGui.separator();
        ImGui.spacing();

        ImGui.text("Existing Joints (" + allJoints.size() + ")");
        ImGui.spacing();

        ImGui.beginChild("##existingJointsList", 0.0f, 0.0f, true);

        Joint jointToDelete = null;

        if (allJoints.isEmpty()) {
            ImGui.textDisabled("No joints created yet.");
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
        if (joint == null)
            return;

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

        allJoints.remove(joint);

        if (onProjectChanged != null) {
            onProjectChanged.run();
        } else if (engine != null) {
            engine.autoSaveProject();
        }

        statusMessage = "Joint deleted: " + joint.getId();
        isErrorStatus = false;
    }

    private void revertPreviewPosition() {
        if (activePreviewChild != null && originalChildPos != null) {
            activePreviewChild.getLocalPosition().set(originalChildPos);
        }
        activePreviewChild = null;
        originalChildPos = null;
    }

    private boolean jointExists(String id, List<Joint> joints) {
        if (joints == null)
            return false;
        for (Joint j : joints) {
            if (j.getId().equals(id))
                return true;
        }
        return false;
    }

    private boolean isAncestor(SceneNode potentialAncestor, SceneNode node) {
        SceneNode current = node.getParent();
        while (current != null) {
            if (current == potentialAncestor) {
                return true;
            }
            current = current.getParent();
        }
        return false;
    }

    private void flattenNodes(SceneNode node, List<SceneNode> list) {
        if (node == null)
            return;
        list.add(node);
        for (SceneNode child : node.getChildren()) {
            flattenNodes(child, list);
        }
    }
}
