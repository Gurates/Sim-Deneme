package rsim2.scene;

import org.joml.Vector3f;

public class Joint {
    private String id;
    private SceneNode parentNode;
    private SceneNode childNode;
    private Vector3f axis;
    private float currentAngleRadians;

    public Joint(String id, SceneNode parentNode, SceneNode childNode, Vector3f axis) {
        this.id = id;
        this.parentNode = parentNode;
        this.childNode = childNode;
        this.axis = new Vector3f(axis).normalize();
        this.currentAngleRadians = 0.0f;
    }

    public void setAngle(float radians) {
        this.currentAngleRadians = radians;
        if (childNode != null) {
            childNode.getLocalRotation().fromAxisAngleRad(axis, radians);
        }
    }

    public String getId() {
        return id;
    }

    public SceneNode getParentNode() {
        return parentNode;
    }

    public SceneNode getChildNode() {
        return childNode;
    }

    public Vector3f getAxis() {
        return axis;
    }

    public float getCurrentAngleRadians() {
        return currentAngleRadians;
    }
}
