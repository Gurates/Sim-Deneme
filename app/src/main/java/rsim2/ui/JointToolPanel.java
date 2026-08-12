package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.joml.Vector3f;
import rsim2.core.Engine;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;

import java.util.ArrayList;
import java.util.List;

public class JointToolPanel {
    private final Engine engine;
    private SceneNode rootNode;
    private List<Joint> allJoints;

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
        this.engine = engine;
        this.rootNode = rootNode;
        this.allJoints = allJoints != null ? allJoints : new ArrayList<>();
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

        ImGui.text("Parent Link");
        ImGui.setNextItemWidth(-1.0f);
        ImGui.combo("##parentCombo", selectedParentIdx, nodeNames);

        ImGui.spacing();

        ImGui.text("Child Link");
        ImGui.setNextItemWidth(-1.0f);
        int prevChildIdx = selectedChildIdx.get();
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

        ImGui.spacing();
        ImGui.text("Rotation Axis");

        boolean isX = selectedAxis.x == 1.0f;
        if (isX) ImGui.pushStyleColor(ImGuiCol.Button, 0.2f, 0.6f, 0.9f, 1.0f);
        if (ImGui.button("X##axisX", 75.0f, 24.0f)) {
            selectedAxis.set(1.0f, 0.0f, 0.0f);
        }
        if (isX) ImGui.popStyleColor();

        ImGui.sameLine();
        boolean isY = selectedAxis.y == 1.0f;
        if (isY) ImGui.pushStyleColor(ImGuiCol.Button, 0.2f, 0.6f, 0.9f, 1.0f);
        if (ImGui.button("Y##axisY", 75.0f, 24.0f)) {
            selectedAxis.set(0.0f, 1.0f, 0.0f);
        }
        if (isY) ImGui.popStyleColor();

        ImGui.sameLine();
        boolean isZ = selectedAxis.z == 1.0f;
        if (isZ) ImGui.pushStyleColor(ImGuiCol.Button, 0.2f, 0.6f, 0.9f, 1.0f);
        if (ImGui.button("Z##axisZ", 75.0f, 24.0f)) {
            selectedAxis.set(0.0f, 0.0f, 1.0f);
        }
        if (isZ) ImGui.popStyleColor();

        ImGui.spacing();
        ImGui.text("Origin Position (Live Preview)");

        ImGui.text("X"); ImGui.sameLine(25.0f); ImGui.setNextItemWidth(-1.0f);
        boolean xChg = ImGui.inputFloat("##origX", originX, 0.05f, 0.5f, "%.2f");

        ImGui.text("Y"); ImGui.sameLine(25.0f); ImGui.setNextItemWidth(-1.0f);
        boolean yChg = ImGui.inputFloat("##origY", originY, 0.05f, 0.5f, "%.2f");

        ImGui.text("Z"); ImGui.sameLine(25.0f); ImGui.setNextItemWidth(-1.0f);
        boolean zChg = ImGui.inputFloat("##origZ", originZ, 0.05f, 0.5f, "%.2f");

        if ((xChg || yChg || zChg) && currentChild != null) {
            currentChild.getLocalPosition().set(originX.get(), originY.get(), originZ.get());
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

                String baseJointId = parentNode.getId() + "_" + childNode.getId() + "_joint";
                String uniqueJointId = baseJointId;
                int counter = 1;
                while (jointExists(uniqueJointId, allJoints)) {
                    uniqueJointId = baseJointId + "_" + counter++;
                }

                Joint newJoint = new Joint(uniqueJointId, parentNode, childNode, new Vector3f(selectedAxis));
                allJoints.add(newJoint);

                String currentPath = engine != null ? engine.getCurrentProjectPath() : null;
                if (currentPath != null && !currentPath.trim().isEmpty()) {
                    try {
                        engine.saveProject(currentPath);
                        statusMessage = "Joint created: " + uniqueJointId + " (Saved)";
                    } catch (Exception e) {
                        statusMessage = "Joint created: " + uniqueJointId + " (Save failed)";
                    }
                } else {
                    statusMessage = "Joint created: " + uniqueJointId;
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

        ImGui.end();
    }

    private void revertPreviewPosition() {
        if (activePreviewChild != null && originalChildPos != null) {
            activePreviewChild.getLocalPosition().set(originalChildPos);
        }
        activePreviewChild = null;
        originalChildPos = null;
    }

    private boolean jointExists(String id, List<Joint> joints) {
        if (joints == null) return false;
        for (Joint j : joints) {
            if (j.getId().equals(id)) return true;
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
        if (node == null) return;
        list.add(node);
        for (SceneNode child : node.getChildren()) {
            flattenNodes(child, list);
        }
    }
}
