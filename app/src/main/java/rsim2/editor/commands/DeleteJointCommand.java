package rsim2.editor.commands;

import org.joml.Quaternionf;
import org.joml.Vector3f;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;

import java.util.List;

public class DeleteJointCommand implements Command {
    private final Joint jointToDelete;
    private final List<Joint> allJointsRef;
    private final SceneNode childNode;
    private final SceneNode oldParent;
    private final int oldChildIndex;
    private final Vector3f oldLocalPosition;
    private final Quaternionf oldLocalRotation;
    private final Vector3f oldLocalScale;
    private final SceneNode rootNode;

    public DeleteJointCommand(Joint jointToDelete, List<Joint> allJointsRef, SceneNode rootNode) {
        this.jointToDelete = jointToDelete;
        this.allJointsRef = allJointsRef;
        this.rootNode = rootNode;

        this.childNode = jointToDelete != null ? jointToDelete.getChildNode() : null;
        this.oldParent = childNode != null ? childNode.getParent() : null;
        this.oldChildIndex = (oldParent != null) ? oldParent.indexOfChild(childNode) : -1;
        this.oldLocalPosition = childNode != null ? new Vector3f(childNode.getLocalPosition()) : new Vector3f();
        this.oldLocalRotation = childNode != null ? new Quaternionf(childNode.getLocalRotation()) : new Quaternionf();
        this.oldLocalScale = childNode != null ? new Vector3f(childNode.getLocalScale()) : new Vector3f(1.0f, 1.0f, 1.0f);
    }

    @Override
    public void execute() {
        if (jointToDelete == null) return;

        if (childNode != null && rootNode != null && childNode.getParent() != rootNode) {
            childNode.reparentPreservingWorldTransform(rootNode);
        }

        if (allJointsRef != null) {
            allJointsRef.remove(jointToDelete);
        }
    }

    @Override
    public void undo() {
        if (jointToDelete == null) return;

        if (childNode != null && oldParent != null) {
            if (childNode.getParent() != null) {
                childNode.getParent().removeChild(childNode);
            }
            oldParent.addChild(oldChildIndex, childNode);
            childNode.getLocalPosition().set(oldLocalPosition);
            childNode.getLocalRotation().set(oldLocalRotation);
            childNode.getLocalScale().set(oldLocalScale);
        }

        if (allJointsRef != null && !allJointsRef.contains(jointToDelete)) {
            allJointsRef.add(jointToDelete);
        }
    }

    @Override
    public String getDescription() {
        return "Delete Joint: " + (jointToDelete != null ? jointToDelete.getId() : "null");
    }
}
