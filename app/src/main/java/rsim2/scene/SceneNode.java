package rsim2.scene;

import org.joml.Matrix4f;
import org.joml.Quaternionf;
import org.joml.Vector3f;
import rsim2.graphics.Mesh;

import java.util.ArrayList;
import java.util.List;

public class SceneNode {
    private String id;
    private Mesh mesh;
    private final Vector3f localPosition;
    private final Quaternionf localRotation;
    private SceneNode parent;
    private final List<SceneNode> children;

    public SceneNode(String id) {
        this(id, null);
    }

    public SceneNode(String id, Mesh mesh) {
        this.id = id;
        this.mesh = mesh;
        this.localPosition = new Vector3f();
        this.localRotation = new Quaternionf();
        this.children = new ArrayList<>();
    }

    public Matrix4f getLocalTransform() {
        return new Matrix4f().translationRotate(localPosition, localRotation);
    }

    public Matrix4f getWorldTransform() {
        if (parent != null) {
            return new Matrix4f(parent.getWorldTransform()).mul(getLocalTransform());
        }
        return getLocalTransform();
    }

    public void addChild(SceneNode child) {
        child.setParent(this);
        children.add(child);
    }

    public String getId() {
        return id;
    }

    public void setId(String id) {
        this.id = id;
    }

    public Mesh getMesh() {
        return mesh;
    }

    public void setMesh(Mesh mesh) {
        this.mesh = mesh;
    }

    public Vector3f getLocalPosition() {
        return localPosition;
    }

    public Quaternionf getLocalRotation() {
        return localRotation;
    }

    public SceneNode getParent() {
        return parent;
    }

    public void setParent(SceneNode parent) {
        this.parent = parent;
    }

    public List<SceneNode> getChildren() {
        return children;
    }
}
