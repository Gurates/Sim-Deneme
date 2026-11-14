package rsim2.motion;

import org.joml.Vector3f;
import rsim2.scene.Joint;
import rsim2.scene.ObjectInteractionController;
import rsim2.scene.SceneNode;

import java.util.List;

public record MotionEvent(float timeSeconds, String objectId, String toolId,
                          float offsetX, float offsetY, float offsetZ) {
    public MotionEvent(float timeSeconds, String objectId, String toolId, Vector3f offset) {
        this(timeSeconds, objectId, toolId, offset.x, offset.y, offset.z);
    }

    void apply(SceneNode root, List<Joint> joints, ObjectInteractionController interactions) {
        SceneNode object = ObjectInteractionController.findNode(root, objectId);
        if (object == null || object == root || object.isRobotPart()) {
            throw new IllegalArgumentException("Motion object is missing: " + objectId);
        }
        if (toolId == null) {
            interactions.release(root, objectId);
        } else {
            SceneNode tool = ObjectInteractionController.findNode(root, toolId);
            if (tool == null) throw new IllegalArgumentException("Motion tool is missing: " + toolId);
            interactions.captureAtContact(root, joints, object, tool, new Vector3f(offsetX, offsetY, offsetZ));
        }
    }
}
