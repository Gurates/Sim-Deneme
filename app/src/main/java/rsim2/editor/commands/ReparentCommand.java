package rsim2.editor.commands;

import org.joml.Quaternionf;
import org.joml.Vector3f;
import rsim2.scene.Joint;
import rsim2.scene.SceneNode;

import java.util.ArrayList;
import java.util.List;

public class ReparentCommand implements Command {
    private final SceneNode child;
    private final SceneNode oldParent;
    private final int oldChildIndex;
    private final Vector3f oldLocalPosition;
    private final Quaternionf oldLocalRotation;
    private final Vector3f oldLocalScale;

    private final SceneNode newParent;
    private final Joint newJoint;
    private final List<Joint> allJointsRef;
    private final List<Joint> replacedOldJoints = new ArrayList<>();

    private Vector3f appliedLocalPosition;
    private Quaternionf appliedLocalRotation;
    private Vector3f appliedLocalScale;

    public ReparentCommand(SceneNode child, SceneNode newParent, Joint newJoint, List<Joint> allJointsRef) {
        this.child = child;
        this.oldParent = child != null ? child.getParent() : null;
        this.oldChildIndex = (oldParent != null) ? oldParent.indexOfChild(child) : -1;
        this.oldLocalPosition = child != null ? new Vector3f(child.getLocalPosition()) : new Vector3f();
        this.oldLocalRotation = child != null ? new Quaternionf(child.getLocalRotation()) : new Quaternionf();
        this.oldLocalScale = child != null ? new Vector3f(child.getLocalScale()) : new Vector3f(1.0f, 1.0f, 1.0f);

        this.newParent = newParent;
        this.newJoint = newJoint;
        this.allJointsRef = allJointsRef;

        if (allJointsRef != null && child != null) {
            for (Joint j : allJointsRef) {
                if (j.getChildNode() == child) {
                    this.replacedOldJoints.add(j);
                }
            }
        }
    }

    @Override
    public void execute() {
        if (child == null || newParent == null) return;

        if (allJointsRef != null) {
            allJointsRef.removeAll(replacedOldJoints);
        }

        if (appliedLocalPosition != null) {
            if (child.getParent() != null) {
                child.getParent().removeChild(child);
            }
            newParent.addChild(child);
            child.getLocalPosition().set(appliedLocalPosition);
            child.getLocalRotation().set(appliedLocalRotation);
            child.getLocalScale().set(appliedLocalScale);
        } else {
            child.reparentPreservingWorldTransform(newParent);
            appliedLocalPosition = new Vector3f(child.getLocalPosition());
            appliedLocalRotation = new Quaternionf(child.getLocalRotation());
            appliedLocalScale = new Vector3f(child.getLocalScale());
        }

        if (allJointsRef != null && newJoint != null && !allJointsRef.contains(newJoint)) {
            allJointsRef.add(newJoint);
        }
    }

    @Override
    public void undo() {
        if (child == null) return;

        if (allJointsRef != null && newJoint != null) {
            allJointsRef.remove(newJoint);
        }

        if (allJointsRef != null) {
            for (Joint j : replacedOldJoints) {
                if (!allJointsRef.contains(j)) {
                    allJointsRef.add(j);
                }
            }
        }

        if (child.getParent() != null) {
            child.getParent().removeChild(child);
        }

        if (oldParent != null) {
            oldParent.addChild(oldChildIndex, child);
        }

        child.getLocalPosition().set(oldLocalPosition);
        child.getLocalRotation().set(oldLocalRotation);
        child.getLocalScale().set(oldLocalScale);
    }

    @Override
    public String getDescription() {
        String pName = newParent != null ? newParent.getId() : "null";
        String cName = child != null ? child.getId() : "null";
        return "Reparent " + cName + " under " + pName;
    }
}
