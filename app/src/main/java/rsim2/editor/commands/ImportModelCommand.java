package rsim2.editor.commands;

import rsim2.editor.SelectionManager;
import rsim2.scene.SceneNode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ImportModelCommand implements Command {
    private final List<SceneNode> importedNodes = new ArrayList<>();
    private final SceneNode parentNode;
    private final SelectionManager selectionManager;

    public ImportModelCommand(SceneNode importedNode, SceneNode parentNode, SelectionManager selectionManager) {
        this(importedNode != null ? Collections.singletonList(importedNode) : Collections.emptyList(), parentNode, selectionManager);
    }

    public ImportModelCommand(List<SceneNode> importedNodes, SceneNode parentNode, SelectionManager selectionManager) {
        if (importedNodes != null) {
            this.importedNodes.addAll(importedNodes);
        }
        this.parentNode = parentNode;
        this.selectionManager = selectionManager;
    }

    @Override
    public void execute() {
        if (parentNode == null) return;
        for (SceneNode node : importedNodes) {
            if (node.getParent() == null) {
                parentNode.addChild(node);
            }
        }
        if (!importedNodes.isEmpty() && selectionManager != null) {
            selectionManager.select(importedNodes.get(0));
        }
    }

    @Override
    public void undo() {
        if (parentNode == null) return;
        for (SceneNode node : importedNodes) {
            if (selectionManager != null && selectionManager.getSelected() == node) {
                selectionManager.clearSelection();
            }
            if (node.getParent() == parentNode) {
                parentNode.removeChild(node);
            }
        }
    }

    @Override
    public String getDescription() {
        if (importedNodes.size() == 1) {
            return "Import Model: " + importedNodes.get(0).getId();
        }
        return "Import " + importedNodes.size() + " Models";
    }
}
