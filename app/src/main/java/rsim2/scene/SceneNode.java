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
    private String sourceMeshPath;
    private final Vector3f localPosition;
    private final Quaternionf localRotation;
    private final Vector3f localScale;
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
        this.localScale = new Vector3f(1.0f, 1.0f, 1.0f);
        this.children = new ArrayList<>();
    }

    public Matrix4f getLocalTransform() {
        return new Matrix4f().translationRotateScale(localPosition, localRotation, localScale);
    }

    public Matrix4f getWorldTransform() {
        if (parent != null) {
            return new Matrix4f(parent.getWorldTransform()).mul(getLocalTransform());
        }
        return getLocalTransform();
    }

    public void reparentPreservingWorldTransform(SceneNode newParent) {
        if (this.parent == newParent) {
            return;
        }

        Matrix4f currentWorld = new Matrix4f(this.getWorldTransform());

        if (this.parent != null) {
            this.parent.removeChild(this);
        }

        if (newParent != null) {
            newParent.addChild(this);
            Matrix4f newParentWorldInverse = new Matrix4f(newParent.getWorldTransform()).invert();
            Matrix4f newLocalTransform = newParentWorldInverse.mul(currentWorld, new Matrix4f());

            newLocalTransform.getTranslation(this.localPosition);
            newLocalTransform.getNormalizedRotation(this.localRotation);
            newLocalTransform.getScale(this.localScale);
        } else {
            currentWorld.getTranslation(this.localPosition);
            currentWorld.getNormalizedRotation(this.localRotation);
            currentWorld.getScale(this.localScale);
        }
    }

    public void setWorldPosition(Vector3f targetWorldPos) {
        if (parent != null) {
            Matrix4f parentWorldInv = new Matrix4f(parent.getWorldTransform()).invert();
            parentWorldInv.transformPosition(targetWorldPos, this.localPosition);
        } else {
            this.localPosition.set(targetWorldPos);
        }
    }

    public void addChild(SceneNode child) {
        child.setParent(this);
        children.add(child);
    }

    public boolean removeChild(SceneNode child) {
        if (child != null && children.remove(child)) {
            child.setParent(null);
            return true;
        }
        return false;
    }

    public void cleanup() {
        if (mesh != null) {
            mesh.cleanup();
        }
        for (SceneNode child : children) {
            child.cleanup();
        }
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

    public String getSourceMeshPath() {
        return sourceMeshPath;
    }

    public void setSourceMeshPath(String sourceMeshPath) {
        this.sourceMeshPath = sourceMeshPath;
    }

    public Vector3f getLocalPosition() {
        return localPosition;
    }

    public Quaternionf getLocalRotation() {
        return localRotation;
    }

    public Vector3f getLocalScale() {
        return localScale;
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
