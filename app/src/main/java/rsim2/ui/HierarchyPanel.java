package rsim2.ui;

import imgui.ImGui;
import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiTreeNodeFlags;
import imgui.flag.ImGuiWindowFlags;
import rsim2.editor.SelectionManager;
import rsim2.scene.SceneNode;

public class HierarchyPanel {
    public static SceneNode draggedNode = null;

    private SceneNode rootNode;
    private final SelectionManager selectionManager;

    public HierarchyPanel(SceneNode rootNode, SelectionManager selectionManager) {
        this.rootNode = rootNode;
        this.selectionManager = selectionManager;
    }

    public void setRootNode(SceneNode rootNode) {
        this.rootNode = rootNode;
    }

    public void render() {
        float displayHeight = ImGui.getIO().getDisplaySizeY();
        float panelWidth = 280.0f;
        float topOffsetY = 40.0f;
        float totalHeight = displayHeight - topOffsetY;
        float hierarchyHeight = totalHeight * 0.5f;

        ImGui.setNextWindowPos(0.0f, topOffsetY, ImGuiCond.Always);
        ImGui.setNextWindowSize(panelWidth, hierarchyHeight, ImGuiCond.Always);

        int flags = ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoCollapse;

        ImGui.begin("Hierarchy", flags);

        if (rootNode != null) {
            renderNode(rootNode);
        }

        ImGui.end();
    }

    private void renderNode(SceneNode node) {
        if (node == null) return;

        int nodeFlags = ImGuiTreeNodeFlags.OpenOnArrow | ImGuiTreeNodeFlags.OpenOnDoubleClick | ImGuiTreeNodeFlags.DefaultOpen;
        if (selectionManager != null && selectionManager.getSelected() == node) {
            nodeFlags |= ImGuiTreeNodeFlags.Selected;
        }

        boolean isLeaf = node.getChildren().isEmpty();
        if (isLeaf) {
            nodeFlags |= ImGuiTreeNodeFlags.Leaf | ImGuiTreeNodeFlags.NoTreePushOnOpen;
            ImGui.treeNodeEx(node.getId(), nodeFlags, node.getId());

            handleDragDrop(node);

            if (ImGui.isItemHovered() && ImGui.isMouseReleased(0)) {
                if (selectionManager != null) {
                    selectionManager.select(node);
                }
            }
        } else {
            boolean nodeOpen = ImGui.treeNodeEx(node.getId(), nodeFlags, node.getId());

            handleDragDrop(node);

            if (ImGui.isItemHovered() && ImGui.isMouseReleased(0)) {
                if (selectionManager != null) {
                    selectionManager.select(node);
                }
            }
            if (nodeOpen) {
                for (SceneNode child : node.getChildren()) {
                    renderNode(child);
                }
                ImGui.treePop();
            }
        }
    }

    private void handleDragDrop(SceneNode node) {
        if (ImGui.beginDragDropSource()) {
            draggedNode = node;
            ImGui.setDragDropPayload("SCENE_NODE", node.getId());
            ImGui.text("Parent: " + node.getId());
            ImGui.endDragDropSource();
        }
    }
}
