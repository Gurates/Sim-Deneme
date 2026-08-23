package rsim2.editor.commands;

import rsim2.editor.SelectionManager;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class DeleteNodeCommand implements Command {
    private final SceneNode nodeToDelete;
    private final SceneNode oldParent;
    private final int oldChildIndex;
    private final List<Joint> allJointsRef;
    private final List<Joint> removedJoints = new ArrayList<>();
    private final SelectionManager selectionManager;

    public DeleteNodeCommand(SceneNode nodeToDelete, List<Joint> allJointsRef, SelectionManager selectionManager) {
        this.nodeToDelete = nodeToDelete;
        this.oldParent = nodeToDelete != null ? nodeToDelete.getParent() : null;
        this.oldChildIndex = (oldParent != null) ? oldParent.indexOfChild(nodeToDelete) : -1;
        this.allJointsRef = allJointsRef;
        this.selectionManager = selectionManager;

        if (nodeToDelete != null && allJointsRef != null) {
            Set<SceneNode> subtree = new HashSet<>();
            collectSubtree(nodeToDelete, subtree);
            for (Joint j : allJointsRef) {
                if (subtree.contains(j.getChildNode()) || subtree.contains(j.getParentNode())) {
                    removedJoints.add(j);
                }
            }
        }
    }

    private void collectSubtree(SceneNode node, Set<SceneNode> out) {
        if (node == null) return;
        out.add(node);
        for (SceneNode child : node.getChildren()) {
            collectSubtree(child, out);
        }
    }

    @Override
    public void execute() {
        if (nodeToDelete == null) return;

        if (selectionManager != null && selectionManager.getSelected() == nodeToDelete) {
            selectionManager.clearSelection();
        }

        if (nodeToDelete.getParent() != null) {
            nodeToDelete.getParent().removeChild(nodeToDelete);
        }

        if (allJointsRef != null) {
            allJointsRef.removeAll(removedJoints);
        }
    }

    @Override
    public void undo() {
        if (nodeToDelete == null) return;

        if (oldParent != null) {
            oldParent.addChild(oldChildIndex, nodeToDelete);
        }

        if (allJointsRef != null) {
            for (Joint j : removedJoints) {
                if (!allJointsRef.contains(j)) {
                    allJointsRef.add(j);
                }
            }
        }

        if (selectionManager != null) {
            selectionManager.select(nodeToDelete);
        }
    }

    @Override
    public String getDescription() {
        return "Delete Node: " + (nodeToDelete != null ? nodeToDelete.getId() : "null");
    }
}
