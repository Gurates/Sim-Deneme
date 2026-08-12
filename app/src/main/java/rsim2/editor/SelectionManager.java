package rsim2.editor;

import rsim2.scene.SceneNode;

public class SelectionManager {
    private SceneNode selectedNode;

    public void select(SceneNode node) {
        this.selectedNode = node;
    }

    public SceneNode getSelected() {
        return selectedNode;
    }

    public void clearSelection() {
        this.selectedNode = null;
    }
}
