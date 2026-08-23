package rsim2.editor.commands;

import org.joml.Vector3f;
import rsim2.scene.SceneNode;

public class TransformCommand implements Command {
    private final SceneNode node;
    private final Vector3f oldPosition;
    private final Vector3f newPosition;

    public TransformCommand(SceneNode node, Vector3f oldPosition, Vector3f newPosition) {
        this.node = node;
        this.oldPosition = new Vector3f(oldPosition);
        this.newPosition = new Vector3f(newPosition);
    }

    @Override
    public void execute() {
        if (node != null) {
            node.getLocalPosition().set(newPosition);
        }
    }

    @Override
    public void undo() {
        if (node != null) {
            node.getLocalPosition().set(oldPosition);
        }
    }

    @Override
    public String getDescription() {
        return "Transform: " + (node != null ? node.getId() : "null");
    }
}
